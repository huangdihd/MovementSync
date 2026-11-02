package xin.bbtt.flight;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

/** One in-flight local computation per controller. No queued source-state backlog and no terminal search. */
public final class LocalGotoController implements AutoCloseable {
    private static final Runnable NO_CANCELLATION = () -> {};
    private static final int HORIZON = 12;
    private static final int RESERVE_TICKS = 6;
    private static final int HANDOFF_BRIDGE_TICKS = 1;
    private static final int ROCKET_SEARCH_BEAM_WIDTH = 4;
    private static final float POWERED_CRUISE_RESERVE_PITCH = -20f;
    private final ExecutorService worker = Executors.newSingleThreadExecutor(r -> {
        Thread thread = new Thread(r, "flight-local-goto");
        thread.setDaemon(true);
        return thread;
    });
    private boolean busy;
    private volatile boolean closed;
    private long lastTick = Long.MIN_VALUE;
    private PhysicsState lastState;
    private long lastEpoch = Long.MIN_VALUE;
    private long lastGeneration = Long.MIN_VALUE;
    private long lastRouteRevision = Long.MIN_VALUE;
    private CompletableFuture<Void> completion = CompletableFuture.completedFuture(null);
    // Protected by the MovementSync monitor so route replacement and publication are atomic.
    private WaypointFlightTarget requestedRoute;
    private long routeRevision;
    // Accessed exclusively by worker. Equal route renewals must not reset the arc-length cursor.
    private LocalAirRoute workerRoute;
    private List<Vector3d> workerPoints;
    private long workerEpoch = Long.MIN_VALUE;
    private long workerGeneration = Long.MIN_VALUE;
    private volatile Result latest;

    public enum Status { SAFE_PREFIX, LANDED, UNPROVEN }
    public record Result(long tick, long generation, long epoch, Status status,
            int prefixTicks, int reserveTicks, boolean rocket, double routeCursor,
            LocalLookaheadSolver.PrefixResult unpowered, LocalLookaheadSolver.PrefixResult powered) {}
    public Result latest() { return latest; }

    /** Prepared controls are not an admission capability: the first airborne handoff
     * replays every outcome from the live source before publishing them. */
    public static final class PreparedLaunch {
        private final List<ElytraFlightController.Decision> controls;
        private final FireworkSchedule.LifetimeBounds lifetime;

        private PreparedLaunch(List<ElytraFlightController.Decision> controls,
                FireworkSchedule.LifetimeBounds lifetime) {
            this.controls = List.copyOf(controls);
            this.lifetime = lifetime;
        }

        public boolean matches(HazardRecovery.Context resources) {
            int[] flights = resources.rocketFlightDurations();
            return !resources.attached() && resources.cooldownTicks() == 0 && flights.length > 0
                    && lifetime.equals(FireworkSchedule.lifetimeBounds(flights[0]));
        }
    }

    /** Follow launch preparation, including fixed lost-target destinations, under its existing entry policy. */
    public static PreparedLaunch prepareLaunch(World world, MovementParameters movement,
            PhysicsState initial, WaypointFlightTarget route, Vector3d goal, float yaw, float pitch,
            HazardRecovery.Context resources) {
        int[] flights = resources.rocketFlightDurations();
        if (resources.attached() || resources.cooldownTicks() > 0 || flights.length == 0) return null;
        var lifetime = FireworkSchedule.lifetimeBounds(flights[0]);
        var selected = new ElytraFlightController.Decision(yaw, pitch, true,
                ElytraFlightController.Phase.CRUISE);
        var episode = prepareRocketEpisode(world, movement, initial, selected, HORIZON,
                lifetime, goal, route, false);
        if (episode.prefix().isEmpty()) return null;
        var proof = rocketTransitionProof(world, movement, initial, episode.prefix(), episode.reserve(),
                episode.bridge(), goal, resources, route);
        if (proof == null || !launchEntryPermitted(world, movement, initial, resources, proof)) return null;
        var controls = new ArrayList<ElytraFlightController.Decision>();
        episode.prefix().forEach(step -> controls.add(step.control()));
        episode.reserve().forEach(step -> controls.add(step.control()));
        episode.bridge().forEach(step -> controls.add(step.control()));
        return new PreparedLaunch(controls, lifetime);
    }

    /** No search at deployment: replay the pre-jump controls and retain the ordinary
     * source, route, lifetime and admission gates. A changed resource cannot renew the proof. */
    public boolean publishLaunch(MovementSync sync, Object owner, FlightControlAdmission.Source source,
            PreparedLaunch launch, WaypointFlightTarget route, Vector3d goal,
            HazardRecovery.Context resources, Runnable accepted) {
        if (launch == null || !launch.matches(resources) || !sync.fireworkRequestReady()) return false;
        World world = sync.getWorld();
        MovementParameters movement = sync.getMovementParameters();
        var corridor = route.corridorSnapshot();
        long revision;
        synchronized (sync) {
            if (closed || !current(sync, source) || !sync.isGliding() || source.state().onGround()) return false;
            revision = observeRoute(corridor);
        }
        var episode = rocketEpisode(world, movement, source.state(), launch.controls, launch.lifetime);
        var proof = rocketTransitionProof(world, movement, source.state(), episode.prefix(), episode.reserve(),
                episode.bridge(), goal, resources, corridor);
        if (proof == null || !launchEntryPermitted(world, movement, source.state(), resources, proof)) return false;
        synchronized (sync) {
            if (closed || routeRevision != revision || sync.getWorld() != world || !current(sync, source)
                    || !movement.equals(sync.getMovementParameters()) || !sync.fireworkRequestReady()
                    || sync.hasAttachedFirework()
                    || !java.util.Arrays.equals(resources.rocketFlightDurations(),
                            sync.inventoryManager.getHotbarFireworkFlightDurations())
                    || !sync.registerForwardFlightEmergency(owner, source,
                            launch.controls.get(0).yaw(), corridor)) return false;
            var first = launch.controls.get(0);
            return sync.publishFlightPrefix(owner, source, episode.prefix(), episode.reserve(), episode.bridge(),
                    corridor, () -> {
                        sync.setYawToward(first.yaw());
                        sync.pitch.set(first.pitch());
                        accepted.run();
                    }, true, launch.lifetime);
        }
    }

    private static boolean launchEntryPermitted(World world, MovementParameters movement,
            PhysicsState initial, HazardRecovery.Context resources, RocketTransitionProof proof) {
        var scan = new DangerousEntryScan(world);
        return proof.branches().stream().map(branch -> scan.firstEntry(branch.states())).distinct()
                .allMatch(entry -> permitsDangerousEntry(world, initial, entry, movement,
                        resources, true, false, true));
    }

    public boolean request(MovementSync sync, Object owner, FlightControlAdmission.Source source,
            WaypointFlightTarget route, Vector3d goal) {
        return request(sync, owner, source, route, goal, false, null);
    }

    public boolean request(MovementSync sync, Object owner, FlightControlAdmission.Source source,
            WaypointFlightTarget route, Vector3d goal, Runnable accepted) {
        return request(sync, owner, source, route, goal, false, accepted);
    }

    public boolean request(MovementSync sync, Object owner, FlightControlAdmission.Source source,
            WaypointFlightTarget route, Vector3d goal, boolean allowedRocket) {
        return request(sync, owner, source, route, goal, allowedRocket, null);
    }

    public boolean request(MovementSync sync, Object owner, FlightControlAdmission.Source source,
            WaypointFlightTarget route, Vector3d goal, boolean allowedRocket, Runnable accepted) {
        return request(sync, owner, source, route, goal, allowedRocket, accepted, null);
    }

    /** Shared local route controller; a tracking observation forbids all normal touchdown candidates. */
    public boolean request(MovementSync sync, Object owner, FlightControlAdmission.Source source,
            WaypointFlightTarget route, Vector3d goal, boolean allowedRocket, Runnable accepted,
            LocalLookaheadSolver.MovingTarget tracking) {
        return request(sync, owner, source, route, goal, allowedRocket, accepted, tracking,
                new HazardRecovery.Context(sync.inventoryManager.getHotbarFireworkFlightDurations(),
                        0, sync.hasAttachedFirework(), sync.getLandingMemory()), true);
    }

    public boolean request(MovementSync sync, Object owner, FlightControlAdmission.Source source,
            WaypointFlightTarget route, Vector3d goal, boolean allowedRocket, Runnable accepted,
            LocalLookaheadSolver.MovingTarget tracking, HazardRecovery.Context resources,
            boolean allowExposedProgress) {
        return request(sync, owner, source, route, goal, allowedRocket, accepted, tracking,
                resources, allowExposedProgress, false);
    }

    /** Explicit opt-in for static lava goto; shared Follow/End callers retain legacy entry proof. */
    public boolean request(MovementSync sync, Object owner, FlightControlAdmission.Source source,
            WaypointFlightTarget route, Vector3d goal, boolean allowedRocket, Runnable accepted,
            LocalLookaheadSolver.MovingTarget tracking, HazardRecovery.Context resources,
            boolean allowExposedProgress, boolean knownReturn) {
        Objects.requireNonNull(sync);
        Objects.requireNonNull(owner);
        Objects.requireNonNull(source);
        Objects.requireNonNull(route);
        Vector3d endpoint = new Vector3d(goal);
        if (!endpoint.isFinite()) throw new IllegalArgumentException("Non-finite goal");
        var snapshot = route.corridorSnapshot();
        synchronized (sync) {
            if (!current(sync, source)) return false;
            long requestRouteRevision = observeRoute(snapshot);
            synchronized (this) {
                if (closed || busy || source.tick() == lastTick && source.epoch() == lastEpoch
                        && source.generation() == lastGeneration
                        && requestRouteRevision == lastRouteRevision
                        && (source.tick() != 0 || source.state().equals(lastState))) return false;
                World world = sync.getWorld();
                MovementParameters movement = sync.getMovementParameters();
                float yaw = sync.yaw.get();
                float pitch = sync.pitch.get();
                boolean attached = sync.hasAttachedFirework();
                float forward = heading(source.state().position(), snapshot.position(), yaw);
                if (!sync.registerForwardFlightEmergency(owner, source, forward, snapshot)) return false;
                busy = true;
                lastTick = source.tick();
                lastState = source.state();
                lastEpoch = source.epoch();
                lastGeneration = source.generation();
                lastRouteRevision = requestRouteRevision;
                var finished = new CompletableFuture<Void>();
                completion = finished;
                worker.execute(() -> {
                    try {
                        compute(sync, owner, source, snapshot, endpoint, world, movement, yaw, pitch,
                                attached, allowedRocket, accepted, tracking, resources, allowExposedProgress,
                                knownReturn, requestRouteRevision);
                    } catch (ObsoleteRequestException ignored) {
                        // Cancellation discards the whole computation, never an empty-proof fallback.
                    } catch (Throwable failure) {
                        synchronized (LocalGotoController.this) { busy = false; }
                        finished.completeExceptionally(failure);
                        return;
                    }
                    synchronized (LocalGotoController.this) { busy = false; }
                    finished.complete(null);
                });
            }
        }
        return true;
    }

    private void compute(MovementSync sync, Object owner, FlightControlAdmission.Source source,
            WaypointFlightTarget route, Vector3d goal, World world, MovementParameters movement,
            float yaw, float pitch, boolean attached, boolean allowedRocket, Runnable accepted,
            LocalLookaheadSolver.MovingTarget tracking, HazardRecovery.Context resources,
            boolean allowExposedProgress, boolean knownReturn, long requestRouteRevision) {
        Runnable validateRequest = () -> validateRequest(sync, source, world, requestRouteRevision);
        validateRequest.run();
        long traceStarted = System.nanoTime();
        var points = route.waypoints();
        if (workerRoute == null || workerEpoch != source.epoch() || workerGeneration != source.generation()
                || !points.equals(workerPoints)) {
            workerRoute = new LocalAirRoute(route);
            workerPoints = points;
            workerEpoch = source.epoch();
            workerGeneration = source.generation();
        } else {
            workerRoute.advanceToLiveSegment(route);
        }
        var lookahead = workerRoute.lookahead(world, source.state());
        validateRequest.run();
        WaypointFlightTarget controlCorridor = route.terrainValidated() ? route : null;
        WaypointFlightTarget rocketCorridor = controlCorridor == null && (tracking != null || knownReturn)
                ? null : route;
        Vector3d forwardPoint = lookahead.isEmpty() ? points.get(points.size() - 1) : lookahead.get(0);
        float forward = heading(source.state().position(), forwardPoint, yaw);
        List<FlightControlAdmission.Step> prefix = List.of();
        List<FlightControlAdmission.Step> reserve = List.of();
        List<FlightControlAdmission.Step> bridge = List.of();
        PhysicsState episodeEnd = source.state();
        AttachmentExpiryProof attachmentExpiryProof = null;
        List<FlightControlAdmission.Step> rejectionHandOver = List.of();
        RocketTransitionProof rocketTransitionProof = null;
        PreparedEpisode unpoweredEpisode = null;
        LocalLookaheadSolver.PrefixResult unpowered = null, powered = null;
        boolean rocket = false;
        boolean terminalArrival = false;
        boolean arrivalApproach = false;
        boolean strictReturnFallMargin = knownReturn && tracking == null;
        boolean requireAirborne = tracking != null || route.partial() || !route.landingIntent();
        Status status = Status.UNPROVEN;
        if (tracking == null && normalLanding(world, source.state(), goal)) {
            status = Status.LANDED;
        } else if (!source.state().onGround()) {
            // Even an empty visible lookahead is not permission to fly blindly. The solver still
            // checks every executable step towards the route endpoint and may return UNPROVEN.
            var anchors = lookahead.isEmpty() ? List.of(forwardPoint) : lookahead;
            // Short speed-dependent steering samples are not stopping destinations.
            // Static cruise measures progress on the remaining route; physical replay
            // still enforces loaded space and the same planner corridor.
            var progressRoute = tracking == null
                    ? points.subList(route.index(), points.size()) : anchors;
            // The live entity owns horizontal urgency; the validated route owns steering
            // and clearance. Replacing route anchors with a moving endpoint makes these two
            // constraints fight while the asynchronous replacement route is still pending.
            var scoringTarget = tracking;
            if (tracking != null) {
                Vector3d routedAim = tracking.position();
                routedAim.y = Math.max(routedAim.y, forwardPoint.y);
                Vector3d horizontalVelocity = tracking.velocity();
                horizontalVelocity.y = 0;
                scoringTarget = new LocalLookaheadSolver.MovingTarget(routedAim, horizontalVelocity);
            }
            boolean supportedGoal = tracking == null && normalLanding(world,
                    new PhysicsState(goal, new Vector3d(), true, 0, PlayerPose.STANDING), goal);
            // Height overshoot is descent, not another cruise leg. A spherical arrival
            // radius lets a supported goal directly below us trigger repeated boosts.
            boolean arriving = tracking == null && (source.state().position().distance(goal) < 32
                    || supportedGoal && source.state().position().y >= goal.y
                    && Math.hypot(source.state().position().x - goal.x,
                            source.state().position().z - goal.z) < 32);
            boolean needsClimb = source.state().position().y < goal.y - 1;
            arrivalApproach = supportedGoal && arriving && !needsClimb;
            boolean preserveFallMargin = strictReturnFallMargin || arrivalApproach;
            LocalLookaheadSolver.PrefixValidator expiryValidator = null;
            if (attached) {
                expiryValidator = candidate -> attachmentExpiryTicks(world, movement, source.state(), candidate,
                        preserveFallMargin, controlCorridor);
            } else if (strictReturnFallMargin) {
                expiryValidator = candidate -> {
                    if (candidate.isEmpty()) return 0;
                    return replay(world, movement, source.state(), candidate.get(0).control(), false,
                            candidate.size(), goal, false, controlCorridor, true).steps().size();
                };
            }
            var solver = new LocalLookaheadSolver();
            var budget = new LocalLookaheadSolver.Budget(600, HORIZON, 25_000_000);
            validateRequest.run();
            unpowered = solver.solvePrefix(anchors, source.state(), world, movement, yaw, pitch,
                    attached, budget, scoringTarget, controlCorridor, expiryValidator, progressRoute);
            validateRequest.run();
            prefix = unpowered.prefix();
            // Attached thrust cannot authorize touchdown. Brake on an exact airborne
            // execution path until the server-observed attachment ends, then publish
            // the terminal landing proof from the resulting physical state.
            terminalArrival = arrivalApproach && !attached;
            boolean lavaMargin = knownReturn && tracking == null && needsLavaMargin(world, source.state());
            var arrival = arrivalApproach && !lavaMargin
                    ? arrival(world, movement, source.state(), goal, attached, forward)
                    : List.<FlightControlAdmission.Step>of();
            // Distance alone does not prove the remaining glide can reach support.
            // Missing arrival controls permit a rescue search, never an unproved request
            // or another boost directly over the landing footprint.
            boolean arrivalNeedsThrust = terminalArrival && arrival.isEmpty()
                    && Math.hypot(source.state().position().x - goal.x,
                            source.state().position().z - goal.z) > 2;
            if (allowedRocket && !attached && accepted != null && (!arriving || needsClimb || arrivalNeedsThrust)
                    && !resources.attached() && resources.cooldownTicks() == 0
                    && resources.rocketFlightDurations().length > 0) {
                validateRequest.run();
                powered = solver.solvePrefix(arrivalNeedsThrust ? List.of(goal) : anchors,
                        source.state(), world, movement, yaw, pitch, true, budget, scoringTarget, controlCorridor,
                        null, arrivalNeedsThrust ? List.of(goal) : progressRoute);
                validateRequest.run();
                if (!powered.prefix().isEmpty()) {
                    // A constant-control horizon is a search seed, not the safety proof
                    // for a maneuvering episode. Pursuit may need to turn or climb before
                    // that seed's horizon ends; the complete lifetime proof decides safety.
                    if (preferPowered(powered.prefix().size(), powered.score(), unpowered)
                            || tracking != null && (powered.score() > unpowered.score()
                                    || pursuitCoastNeedsRenewal(world, movement, source.state(), unpowered,
                                            resources, goal, controlCorridor, validateRequest))
                            || arrivalNeedsThrust) {
                        prefix = withFirstRocket(powered.prefix(), true);
                        rocket = true;
                    }
                }
            }
            // Preserve maneuvering margin over lava rather than matching a dangerously
            // low route at minimum speed. Follow/End never opt in to this policy.
            if (lavaMargin) terminalArrival = false;
            if (lavaMargin && (attached || allowedRocket && accepted != null)) {
                // Existing thrust still needs escape steering: the ordinary solver may
                // track a descending route straight back toward lava. Do not stack a
                // rocket; retain the escape's same-control no-thrust commitment bound.
                var escape = poweredEscape(world, movement, source.state(), forward, goal,
                        preserveFallMargin);
                if (!escape.isEmpty()) {
                    prefix = escape;
                    rocket = !attached;
                }
            }
            if (!arrival.isEmpty()) {
                prefix = arrival;
                rocket = false;
            }
            if (rocket && !unpowered.prefix().isEmpty()) {
                unpoweredEpisode = prepareEpisode(world, movement, source.state(),
                        unpowered.prefix().get(0).control(), false, false,
                        unpowered.prefix().size(), goal, terminalArrival,
                        tracking == null && (!arrivalApproach || terminalArrival),
                        terminalArrival ? null : controlCorridor, preserveFallMargin, validateRequest);
                if (unpoweredEpisode.prefix().isEmpty()) unpoweredEpisode = null;
            }
            // Keep a terminal prefix within the admission revalidation horizon. Every later
            // cursor then reaches the same checkpoint before its exact landing/recovery proof.
            if (terminalArrival && !rocket && !prefix.isEmpty()
                    && prefix.get(0).control().phase() == ElytraFlightController.Phase.LANDING
                    && prefix.size() > RESERVE_TICKS)
                prefix = List.copyOf(prefix.subList(0, RESERVE_TICKS));
            // A non-terminal prefix is publishable only when its endpoint owns a complete,
            // fixed continuation. Shorten forward commitment instead of spending the final
            // locally-safe tick and hoping the next asynchronous worker finds a way out.
            if (!prefix.isEmpty()) {
                WaypointFlightTarget episodeCorridor = terminalArrival ? null : controlCorridor;
                if (rocket) episodeCorridor = rocketCorridor;
                // A second rocket search adds worker latency to every strict refusal. Only a
                // static lava escape, whose unpowered alternative is itself sinking, hands over.
                var episode = prepareEpisode(world, movement, source.state(), prefix.get(0).control(),
                        attached || rocket, rocket, prefix.size(),
                        rocket ? FireworkSchedule.lifetimeBounds(resources.rocketFlightDurations()[0]) : null,
                        goal, terminalArrival && !rocket,
                        tracking == null && (!arrivalApproach || terminalArrival),
                        episodeCorridor, preserveFallMargin, validateRequest, tracking,
                        tracking == null && lavaMargin);
                prefix = episode.prefix();
                reserve = episode.reserve();
                bridge = episode.bridge();
                episodeEnd = episode.end();
                rejectionHandOver = episode.rejectionHandOver();
                if (!prefix.isEmpty()) status = Status.SAFE_PREFIX;
            }
            if (prefix.isEmpty() && rocket && unpoweredEpisode != null) {
                prefix = unpoweredEpisode.prefix();
                reserve = unpoweredEpisode.reserve();
                bridge = unpoweredEpisode.bridge();
                episodeEnd = unpoweredEpisode.end();
                rocket = false;
                status = Status.SAFE_PREFIX;
            }
            if (prefix.isEmpty()) {
                // The generic solver deliberately excludes ground contact; bounded goal-only
                // braking may include a harmless supported touchdown, never arbitrary ground.
                var brake = new ElytraFlightController.Decision(forward, -45, false,
                        tracking == null ? ElytraFlightController.Phase.LANDING
                                : ElytraFlightController.Phase.CRUISE);
                var fallback = prepareEpisode(world, movement, source.state(), brake,
                        attached, false, RESERVE_TICKS, goal, tracking == null,
                        tracking == null, controlCorridor, preserveFallMargin, validateRequest);
                prefix = fallback.prefix();
                reserve = fallback.reserve();
                bridge = fallback.bridge();
                episodeEnd = fallback.end();
                rocket = false;
                if (!prefix.isEmpty()) status = Status.SAFE_PREFIX;
            }
        }
        if (!prefix.isEmpty() && attached) {
            var selected = prefix.get(0).control();
            int maximumPrefixTicks = prefix.size();
            for (int prefixTicks = maximumPrefixTicks; prefixTicks > 0; prefixTicks--) {
                var episode = prepareEpisode(world, movement, source.state(), selected,
                        true, false, prefixTicks, goal, false,
                        tracking == null && !arrivalApproach, controlCorridor,
                        strictReturnFallMargin || arrivalApproach, validateRequest);
                if (episode.prefix().isEmpty()) continue;
                var proof = attachmentExpiryProof(world, movement, source.state(), episode.prefix(),
                        episode.reserve(), episode.bridge(), goal,
                        strictReturnFallMargin || arrivalApproach, controlCorridor, validateRequest);
                if (proof == null) {
                    continue;
                }
                prefix = episode.prefix();
                reserve = episode.reserve();
                bridge = episode.bridge();
                attachmentExpiryProof = proof;
                var poweredBranch = proof.branches().get(proof.branches().size() - 1).states();
                episodeEnd = poweredBranch.get(poweredBranch.size() - 1);
                break;
            }
            if (attachmentExpiryProof == null) {
                prefix = List.of();
                reserve = List.of();
                bridge = List.of();
                status = Status.UNPROVEN;
            }
        }
        if (!prefix.isEmpty() && rocket) {
            // Powered branches remain in planner geometry. A rejected request may instead close
            // on separately proved safe support; admission replays that branch from live state.
            rocketTransitionProof = rocketTransitionProof(world, movement, source.state(), prefix,
                    reserve, bridge, goal, resources, rocketCorridor, strictReturnFallMargin, validateRequest,
                    rejectionHandOver);
            if (rocketTransitionProof == null) {
                if (unpoweredEpisode != null) {
                    prefix = unpoweredEpisode.prefix();
                    reserve = unpoweredEpisode.reserve();
                    bridge = unpoweredEpisode.bridge();
                    episodeEnd = unpoweredEpisode.end();
                    rocket = false;
                    status = Status.SAFE_PREFIX;
                } else {
                    prefix = List.of();
                    reserve = List.of();
                    bridge = List.of();
                    rocket = false;
                    status = Status.UNPROVEN;
                }
            }
        }
        validateRequest.run();
        var publicationSource = source;
        var publicationResources = resources;
        if (!prefix.isEmpty() && tracking != null && !rocket && !attached) {
            var liveSource = FlightControlAdmission.snapshot(sync);
            if (liveSource.tick() > source.tick()) {
                // A six-tick live check cannot renew an entire source-time coast.
                // Rebuild its fixed reserve from the current body before publication;
                // the original request still owns the route/age/cancellation gates.
                var liveEpisode = prepareEpisode(world, movement, liveSource.state(),
                        prefix.get(0).control(), false, false, prefix.size(), goal,
                        false, false, controlCorridor, false, validateRequest);
                prefix = liveEpisode.prefix();
                reserve = liveEpisode.reserve();
                bridge = liveEpisode.bridge();
                episodeEnd = liveEpisode.end();
                publicationSource = liveSource;
                publicationResources = resources.afterTicks((int) (liveSource.tick() - source.tick()), false);
                if (prefix.isEmpty()) status = Status.UNPROVEN;
            }
        }
        if (!prefix.isEmpty() && tracking != null && rocket && rocketTransitionProof != null) {
            var liveSource = FlightControlAdmission.snapshot(sync);
            if (liveSource.tick() > source.tick()) {
                // Admission replays every lifetime branch from the live body inside the
                // published witness. A source-time witness is already behind that body, so
                // re-prove the same fixed controls from it; no search, and no gate skipped.
                var controls = new ArrayList<ElytraFlightController.Decision>(
                        prefix.size() + reserve.size() + bridge.size());
                prefix.forEach(step -> controls.add(step.control()));
                reserve.forEach(step -> controls.add(step.control()));
                bridge.forEach(step -> controls.add(step.control()));
                var liveResources = resources.afterTicks((int) (liveSource.tick() - source.tick()), false);
                var liveEpisode = rocketEpisode(world, movement, liveSource.state(), controls,
                        FireworkSchedule.lifetimeBounds(resources.rocketFlightDurations()[0]));
                var liveProof = rocketTransitionProof(world, movement, liveSource.state(),
                        liveEpisode.prefix(), liveEpisode.reserve(), liveEpisode.bridge(), goal,
                        liveResources, rocketCorridor, strictReturnFallMargin, validateRequest, List.of());
                if (liveProof == null) {
                    prefix = List.of();
                    reserve = List.of();
                    bridge = List.of();
                    rocket = false;
                    rocketTransitionProof = null;
                    status = Status.UNPROVEN;
                } else {
                    prefix = liveEpisode.prefix();
                    reserve = liveEpisode.reserve();
                    bridge = liveEpisode.bridge();
                    episodeEnd = liveEpisode.end();
                    rocketTransitionProof = liveProof;
                    publicationSource = liveSource;
                    publicationResources = liveResources;
                }
            }
        }
        if (!prefix.isEmpty()) {
            boolean permitted;
            DangerousEntry rejectedEntry = null;
            var entryScan = new DangerousEntryScan(world);
            if (attachmentExpiryProof != null) {
                permitted = attachmentExpiryProof.branches().stream()
                        .map(branch -> entryScan.firstEntry(branch.states())).distinct()
                        .allMatch(entry -> permitsDangerousEntry(world, source.state(), entry,
                                movement, resources, allowExposedProgress, knownReturn, false));
            } else if (rocketTransitionProof != null) {
                var entrySource = publicationSource;
                var entryResources = publicationResources;
                permitted = rocketTransitionProof.branches().stream()
                        .map(branch -> entryScan.firstEntry(branch.states())).distinct()
                        .allMatch(entry -> permitsDangerousEntry(world, entrySource.state(), entry,
                                movement, entryResources, allowExposedProgress, knownReturn, true));
            } else {
                rejectedEntry = firstDangerousEntry(world, movement, publicationSource.state(), prefix, reserve,
                        episodeEnd);
                permitted = permitsDangerousEntry(world, publicationSource.state(), rejectedEntry,
                        movement, publicationResources, allowExposedProgress, knownReturn, false);
            }
            if (!permitted) {
                prefix = List.of();
                reserve = List.of();
                bridge = List.of();
                rocket = false;
                status = Status.UNPROVEN;
            }
        }
        synchronized (sync) {
            boolean controllerOpen = !closed;
            boolean routeCurrent = routeRevision == requestRouteRevision;
            boolean worldCurrent = sync.getWorld() == world;
            boolean sourceCurrent = current(sync, source);
            boolean emergencyCurrent = controllerOpen && routeCurrent && worldCurrent && sourceCurrent
                    && sync.refineForwardFlightEmergency(owner, source, forward, route);
            if (Boolean.getBoolean("traceAdmission"))
                System.err.println("WORKER_PHASE source=" + source.tick() + " live=" + sync.physicsTickVersion()
                        + " position=" + source.state().position() + " elapsed=" + (System.nanoTime() - traceStarted) / 1000000
                        + " routeCurrent=" + routeCurrent + " sourceCurrent=" + sourceCurrent
                        + " emergency=" + emergencyCurrent + " status=" + status + " rocket=" + rocket
                        + " unpowered=" + (unpowered == null ? null : unpowered.prefix().size() + "/" + unpowered.score())
                        + " powered=" + (powered == null ? null : powered.prefix().size() + "/" + powered.score())
                        + " selected=" + (prefix.isEmpty() ? null : prefix.get(0).control())
                        + " strict=" + strictReturnFallMargin);
            if (closed || !routeCurrent || !worldCurrent || !sourceCurrent || !emergencyCurrent) return;
            if (publicationSource != source && (sync.hasAttachedFirework()
                    || !sync.rebaseForwardFlightEmergency(owner, source, publicationSource, route))) return;
            if (!prefix.isEmpty()) {
                var selected = prefix.get(0).control();
                // Use the admitted heading for the use-item packet, including return turns.
                // Keep planner geometry aligned with the emergency capability. Rejected rocket
                // requests are re-proved from live state instead of replacing this route.
                WaypointFlightTarget admissionRoute = route;
                if (arrivalApproach && !rocket) {
                    admissionRoute = attachmentExpiryProof == null
                            ? executionCorridor(world, movement, prefix, reserve,
                                    bridge, false, goal, terminalArrival)
                            : attachmentExpiryProof.corridor();
                } else if (controlCorridor == null) {
                    if (rocketTransitionProof != null)
                        admissionRoute = rocketTransitionProof.corridor();
                    else if (attachmentExpiryProof != null)
                        admissionRoute = attachmentExpiryProof.corridor();
                    else if (tracking != null)
                        admissionRoute = executionCorridor(world, movement, prefix, reserve,
                                bridge, false, goal, false);
                }
                boolean admissionRequiresAirborne = rocket || !terminalArrival
                        && (requireAirborne || arrivalApproach);
                var rocketLifetime = rocket
                        ? FireworkSchedule.lifetimeBounds(resources.rocketFlightDurations()[0]) : null;
                sync.publishFlightPrefix(owner, publicationSource, prefix, reserve, bridge,
                        admissionRoute, route, rocket ? () -> {
                    sync.setYawToward(selected.yaw());
                    sync.pitch.set(selected.pitch());
                    accepted.run();
                } : () -> {}, admissionRequiresAirborne, rocketLifetime, strictReturnFallMargin,
                        rocket ? rejectionHandOver : List.of());
            }
            if (sync.getFlightScopeRouter().isShadowEnabled()
                    && (latest == null || source.tick() - latest.tick() >= 10 || status != latest.status()))
                MovementSync.getLogger().info("Local goto tick={} status={} prefix={} rocket={} noVisibleAim={} unpoweredRejects={} poweredRejects={} maneuver={}",
                        source.tick(), status, prefix.size(), rocket, lookahead.isEmpty(),
                        unpowered == null ? null : unpowered.rejections(), powered == null ? null : powered.rejections(),
                        !prefix.isEmpty() && prefix.get(0).control().phase() == ElytraFlightController.Phase.LANDING
                                ? "BOUNDED_GOAL_ARRIVAL" : "FORWARD_ROUTE");
            latest = new Result(source.tick(), source.generation(), source.epoch(), status,
                    prefix.size(), reserve.size(), rocket, workerRoute.cursor(), unpowered, powered);
        }
    }

    private static boolean pursuitCoastNeedsRenewal(World world, MovementParameters movement,
            PhysicsState initial, LocalLookaheadSolver.PrefixResult unpowered,
            HazardRecovery.Context resources, Vector3d goal, WaypointFlightTarget corridor,
            Runnable validateRequest) {
        if (unpowered.prefix().isEmpty()) return true;
        int proofTicks = FireworkSchedule.lifetimeBounds(resources.rocketFlightDurations()[0])
                .maximumPoweredTicks() + 1 + HANDOFF_BRIDGE_TICKS;
        // Compare time horizons before rejecting a maneuvering search for a poor
        // straight-thrust score. A short safe coast can otherwise spend the height
        // needed to prove rejection and expiry for the next rocket request.
        var coast = replay(world, movement, initial, unpowered.prefix().get(0).control(),
                false, proofTicks, goal, false, corridor, false, validateRequest);
        return coast.steps().size() != proofTicks;
    }

    record PreparedEpisode(List<FlightControlAdmission.Step> prefix,
                           List<FlightControlAdmission.Step> reserve,
                           List<FlightControlAdmission.Step> bridge,
                           PhysicsState end,
                           List<FlightControlAdmission.Step> rejectionHandOver) {
        PreparedEpisode {
            prefix = List.copyOf(prefix);
            reserve = List.copyOf(reserve);
            bridge = List.copyOf(bridge);
            rejectionHandOver = List.copyOf(rejectionHandOver);
        }
        PreparedEpisode(List<FlightControlAdmission.Step> prefix,
                List<FlightControlAdmission.Step> reserve,
                List<FlightControlAdmission.Step> bridge, PhysicsState end) {
            this(prefix, reserve, bridge, end, List.of());
        }
    }

    /** Find the longest forward commitment whose endpoint has a complete fixed
     * reserve and another complete fixed bridge. The bridge is proof-only: it keeps
     * the reserve endpoint controllable without extending the admitted lease.
     * A short reserve or bridge is accepted only when it closes on the goal.
     */
    static PreparedEpisode prepareEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected,
            boolean powered, boolean rocket, int maximumPrefixTicks, Vector3d goal,
            boolean prefixLanding, boolean reserveLanding, WaypointFlightTarget corridor) {
        return prepareEpisode(world, movement, initial, selected, powered, rocket,
                maximumPrefixTicks, null, goal, prefixLanding, reserveLanding, corridor);
    }

    private static PreparedEpisode prepareEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected,
            boolean powered, boolean rocket, int maximumPrefixTicks, Vector3d goal,
            boolean prefixLanding, boolean reserveLanding, WaypointFlightTarget corridor,
            boolean preserveFallMargin, Runnable validateRequest) {
        return prepareEpisode(world, movement, initial, selected, powered, rocket,
                maximumPrefixTicks, null, goal, prefixLanding, reserveLanding, corridor,
                preserveFallMargin, validateRequest);
    }

    static PreparedEpisode prepareEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected,
            boolean powered, boolean rocket, int maximumPrefixTicks,
            FireworkSchedule.LifetimeBounds rocketLifetime, Vector3d goal,
            boolean prefixLanding, boolean reserveLanding, WaypointFlightTarget corridor) {
        return prepareEpisode(world, movement, initial, selected, powered, rocket,
                maximumPrefixTicks, rocketLifetime, goal, prefixLanding, reserveLanding, corridor, false);
    }

    static PreparedEpisode prepareEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected,
            boolean powered, boolean rocket, int maximumPrefixTicks,
            FireworkSchedule.LifetimeBounds rocketLifetime, Vector3d goal,
            boolean prefixLanding, boolean reserveLanding, WaypointFlightTarget corridor,
            boolean preserveFallMargin) {
        return prepareEpisode(world, movement, initial, selected, powered, rocket,
                maximumPrefixTicks, rocketLifetime, goal, prefixLanding, reserveLanding, corridor,
                preserveFallMargin, NO_CANCELLATION);
    }

    private static PreparedEpisode prepareEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected,
            boolean powered, boolean rocket, int maximumPrefixTicks,
            FireworkSchedule.LifetimeBounds rocketLifetime, Vector3d goal,
            boolean prefixLanding, boolean reserveLanding, WaypointFlightTarget corridor,
            boolean preserveFallMargin, Runnable validateRequest) {
        return prepareEpisode(world, movement, initial, selected, powered, rocket, maximumPrefixTicks,
                rocketLifetime, goal, prefixLanding, reserveLanding, corridor, preserveFallMargin,
                validateRequest, null);
    }

    private static PreparedEpisode prepareEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected,
            boolean powered, boolean rocket, int maximumPrefixTicks,
            FireworkSchedule.LifetimeBounds rocketLifetime, Vector3d goal,
            boolean prefixLanding, boolean reserveLanding, WaypointFlightTarget corridor,
            boolean preserveFallMargin, Runnable validateRequest, LocalLookaheadSolver.MovingTarget tracking) {
        return prepareEpisode(world, movement, initial, selected, powered, rocket, maximumPrefixTicks,
                rocketLifetime, goal, prefixLanding, reserveLanding, corridor, preserveFallMargin,
                validateRequest, tracking, false);
    }

    private static PreparedEpisode prepareEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected,
            boolean powered, boolean rocket, int maximumPrefixTicks,
            FireworkSchedule.LifetimeBounds rocketLifetime, Vector3d goal,
            boolean prefixLanding, boolean reserveLanding, WaypointFlightTarget corridor,
            boolean preserveFallMargin, Runnable validateRequest, LocalLookaheadSolver.MovingTarget tracking,
            boolean rejectionHandOver) {
        validateRequest.run();
        if (rocketLifetime != null)
            return prepareRocketEpisode(world, movement, initial, selected, maximumPrefixTicks,
                    rocketLifetime, goal, corridor, preserveFallMargin, validateRequest, tracking,
                    rejectionHandOver);
        for (int prefixTicks = maximumPrefixTicks; prefixTicks > 0; prefixTicks--) {
            validateRequest.run();
            var trajectory = replay(world, movement, initial, selected, powered,
                    prefixTicks, goal, prefixLanding, corridor, preserveFallMargin, validateRequest);
            if (trajectory.steps().size() != prefixTicks) {
                continue;
            }
            var prefix = withFirstRocket(trajectory.steps(), rocket);
            if (trajectory.end().onGround()) {
                if (prefixLanding && normalLanding(world, trajectory.end(), goal))
                    return new PreparedEpisode(prefix, List.of(), List.of(), trajectory.end());
                continue;
            }
            float continuationPitch = powered
                    && selected.phase() == ElytraFlightController.Phase.CRUISE
                    && selected.pitch() < POWERED_CRUISE_RESERVE_PITCH
                    ? POWERED_CRUISE_RESERVE_PITCH
                    : selected.pitch();
            var continuationControl = new ElytraFlightController.Decision(
                    selected.yaw(), continuationPitch, false, selected.phase());
            var continuation = replay(world, movement, trajectory.end(), continuationControl, powered,
                    RESERVE_TICKS, goal, reserveLanding, corridor, preserveFallMargin, validateRequest);
            boolean terminal = continuation.end().onGround()
                    && reserveLanding && normalLanding(world, continuation.end(), goal);
            if (terminal)
                return new PreparedEpisode(prefix, continuation.steps(), List.of(), continuation.end());
            if (continuation.steps().size() != RESERVE_TICKS) {
                continue;
            }
            var bridgeControls = selected.equals(continuationControl)
                    ? List.of(continuationControl)
                    : List.of(new ElytraFlightController.Decision(selected.yaw(), selected.pitch(),
                            false, selected.phase()), continuationControl);
            for (var bridgeControl : bridgeControls) {
                var bridge = replay(world, movement, continuation.end(), bridgeControl, powered,
                        HANDOFF_BRIDGE_TICKS, goal, reserveLanding, corridor, preserveFallMargin, validateRequest);
                boolean bridgeTerminal = bridge.end().onGround()
                        && reserveLanding && normalLanding(world, bridge.end(), goal);
                if (bridgeTerminal || bridge.steps().size() == HANDOFF_BRIDGE_TICKS)
                    return new PreparedEpisode(prefix, continuation.steps(), bridge.steps(),
                            continuation.end());
            }
        }
        return new PreparedEpisode(List.of(), List.of(), List.of(), initial);
    }

    private record RocketBranch(int lifetime, int acknowledgementDelay,
                                PhysicsState state, boolean landed) {}
    private record RocketSearchTransition(PhysicsState state, boolean powered) {}
    private record RocketSearchNode(List<ElytraFlightController.Decision> controls,
                                    List<RocketBranch> branches, double score) {}

    private static PreparedEpisode prepareRocketEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected, int maximumPrefixTicks,
            FireworkSchedule.LifetimeBounds lifetime, Vector3d goal,
            WaypointFlightTarget corridor, boolean preserveFallMargin) {
        return prepareRocketEpisode(world, movement, initial, selected, maximumPrefixTicks,
                lifetime, goal, corridor, preserveFallMargin, NO_CANCELLATION);
    }

    private static PreparedEpisode prepareRocketEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected, int maximumPrefixTicks,
            FireworkSchedule.LifetimeBounds lifetime, Vector3d goal,
            WaypointFlightTarget corridor, boolean preserveFallMargin, Runnable validateRequest) {
        return prepareRocketEpisode(world, movement, initial, selected, maximumPrefixTicks,
                lifetime, goal, corridor, preserveFallMargin, validateRequest, null);
    }

    static PreparedEpisode prepareRocketEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected, int maximumPrefixTicks,
            FireworkSchedule.LifetimeBounds lifetime, Vector3d goal,
            WaypointFlightTarget corridor, boolean preserveFallMargin,
            LocalLookaheadSolver.MovingTarget tracking) {
        return prepareRocketEpisode(world, movement, initial, selected, maximumPrefixTicks,
                lifetime, goal, corridor, preserveFallMargin, NO_CANCELLATION, tracking);
    }

    private static PreparedEpisode prepareRocketEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected, int maximumPrefixTicks,
            FireworkSchedule.LifetimeBounds lifetime, Vector3d goal,
            WaypointFlightTarget corridor, boolean preserveFallMargin, Runnable validateRequest,
            LocalLookaheadSolver.MovingTarget tracking) {
        return prepareRocketEpisode(world, movement, initial, selected, maximumPrefixTicks, lifetime,
                goal, corridor, preserveFallMargin, validateRequest, tracking, false);
    }

    /** Strict search first: the rocket controls stay safe without thrust. Only when none
     * do may rejection, once observable, hand over to a proved unpowered reserve held to
     * the same horizon as an ordinary unpowered lease. */
    private static PreparedEpisode prepareRocketEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected, int maximumPrefixTicks,
            FireworkSchedule.LifetimeBounds lifetime, Vector3d goal,
            WaypointFlightTarget corridor, boolean preserveFallMargin, Runnable validateRequest,
            LocalLookaheadSolver.MovingTarget tracking, boolean rejectionHandOver) {
        var strict = searchRocketEpisode(world, movement, initial, selected, maximumPrefixTicks,
                lifetime, goal, corridor, preserveFallMargin, validateRequest, tracking, null);
        if (!strict.prefix().isEmpty() || !rejectionHandOver) return strict;
        return searchRocketEpisode(world, movement, initial, selected, maximumPrefixTicks,
                lifetime, goal, corridor, preserveFallMargin, validateRequest, tracking, selected.yaw());
    }

    private static PreparedEpisode searchRocketEpisode(World world, MovementParameters movement,
            PhysicsState initial, ElytraFlightController.Decision selected, int maximumPrefixTicks,
            FireworkSchedule.LifetimeBounds lifetime, Vector3d goal,
            WaypointFlightTarget corridor, boolean preserveFallMargin, Runnable validateRequest,
            LocalLookaheadSolver.MovingTarget tracking, Float handOverYaw) {
        validateRequest.run();
        int executableTicks = lifetime.maximumPoweredTicks() + 1;
        int totalProofTicks = executableTicks + HANDOFF_BRIDGE_TICKS;
        if (executableTicks <= RESERVE_TICKS || maximumPrefixTicks <= 0)
            return new PreparedEpisode(List.of(), List.of(), List.of(), initial);
        // The active segment owns clearance altitude; the final goal owns horizontal
        // progress. A short partial segment must not make rocket search brake early.
        Vector3d searchGoal = corridor == null ? new Vector3d(goal) : corridor.goal();
        if (corridor != null && corridor.terrainValidated()) searchGoal.y = corridor.position().y;
        double routeAltitude = searchGoal.y;
        // A rocket episode is airborne even for static goto. A supported destination
        // guides horizontal progress; its ground height is not a cruise altitude.
        var localSupport = new Vector3d(initial.position().x, searchGoal.y, initial.position().z);
        if (normalLanding(world, new PhysicsState(localSupport, new Vector3d(), true,
                0, PlayerPose.STANDING), localSupport)) searchGoal.y += 4;
        var fixed = new ElytraFlightController.Decision(selected.yaw(), selected.pitch(), false,
                selected.phase());
        // Most static takeoffs need a short acceleration followed by level cruise,
        // not a combinatorial turn search. A complete all-outcome proof is still
        // mandatory; the fast path changes preparation cost, never admission gates.
        if (tracking == null && (searchGoal.y > routeAltitude
                || Math.hypot(initial.velocity().x, initial.velocity().z) < .3)) {
            var cruise = new ElytraFlightController.Decision(selected.yaw(), 0, false, selected.phase());
            var directControls = new ArrayList<ElytraFlightController.Decision>(totalProofTicks);
            for (int tick = 0; tick < totalProofTicks; tick++)
                directControls.add(tick < 4 ? fixed : cruise);
            if (validRocketControls(world, movement, initial, directControls, lifetime, corridor,
                    preserveFallMargin, validateRequest, handOverYaw))
                return rocketEpisode(world, movement, initial, directControls, lifetime, corridor,
                        preserveFallMargin, validateRequest, handOverYaw);
        }
        var controls = List.of(fixed);
        boolean rejectionMustStayAirborne = DangerousEntryGate.exposed(world, initial);
        var initialBranches = rocketSearchBranches(initial, lifetime, handOverYaw != null);
        var branches = advanceRocketSearchBranches(world, movement,
                initialBranches, fixed, 0, corridor, preserveFallMargin, validateRequest, handOverYaw);
        if (branches == null)
            return new PreparedEpisode(List.of(), List.of(), List.of(), initial);
        var nodes = List.of(new RocketSearchNode(controls, branches,
                rocketSearchScore(branches, searchGoal, fixed, corridor, tracking, controls.size())));
        while (!nodes.isEmpty() && nodes.get(0).controls().size() < totalProofTicks) {
            validateRequest.run();
            int tick = nodes.get(0).controls().size();
            int segmentTicks = Math.min(3, totalProofTicks - tick);
            var next = new ArrayList<RocketSearchNode>();
            for (var node : nodes) {
                for (var control : rocketControlBank(selected,
                        node.controls().get(node.controls().size() - 1))) {
                    validateRequest.run();
                    var advanced = node.branches();
                    for (int offset = 0; offset < segmentTicks && advanced != null; offset++) {
                        advanced = advanceRocketSearchBranches(world, movement, advanced, control,
                                tick + offset, corridor, preserveFallMargin, validateRequest, handOverYaw);
                    }
                    if (advanced == null) continue;
                    var candidateControls = new ArrayList<>(node.controls());
                    for (int offset = 0; offset < segmentTicks; offset++)
                        candidateControls.add(control);
                    double score = rocketSearchScore(advanced, searchGoal, control, corridor,
                            tracking, candidateControls.size());
                    next.add(new RocketSearchNode(List.copyOf(candidateControls), advanced, score));
                }
            }
            next.sort(java.util.Comparator.comparingDouble(RocketSearchNode::score));
            nodes = diverseRocketNodes(next, rejectionMustStayAirborne);
        }
        for (var candidate : nodes) {
            validateRequest.run();
            if (!validRocketControls(world, movement, initial, candidate.controls(), lifetime,
                    corridor, preserveFallMargin, validateRequest, handOverYaw)) continue;
            return rocketEpisode(world, movement, initial, candidate.controls(), lifetime, corridor,
                    preserveFallMargin, validateRequest, handOverYaw);
        }
        return new PreparedEpisode(List.of(), List.of(), List.of(), initial);
    }

    private static PreparedEpisode rocketEpisode(World world, MovementParameters movement,
            PhysicsState initial, List<ElytraFlightController.Decision> controls,
            FireworkSchedule.LifetimeBounds lifetime, WaypointFlightTarget corridor,
            boolean preserveFallMargin, Runnable validateRequest, Float handOverYaw) {
        var episode = rocketEpisode(world, movement, initial, controls, lifetime);
        if (handOverYaw == null) return episode;
        var observed = rejectionObservationState(world, movement, initial, controls);
        var handOver = observed == null ? List.<FlightControlAdmission.Step>of()
                : rejectionHandOver(world, movement, observed, handOverYaw, corridor,
                        preserveFallMargin, validateRequest);
        return new PreparedEpisode(episode.prefix(), episode.reserve(), episode.bridge(),
                episode.end(), handOver);
    }

    /** No-thrust state at the tick rejection becomes observable, or null when the
     * rocket controls themselves do not stay airborne that long. */
    private static PhysicsState rejectionObservationState(World world, MovementParameters movement,
            PhysicsState initial, List<ElytraFlightController.Decision> controls) {
        if (controls.size() < FlightControlAdmission.REJECTED_ROCKET_OBSERVATION_TICKS) return null;
        PhysicsState state = initial;
        for (int tick = 0; tick < FlightControlAdmission.REJECTED_ROCKET_OBSERVATION_TICKS; tick++) {
            var control = controls.get(tick);
            var result = new GlidingPhysics(control.yaw(), control.pitch(), false,
                    PlayerInput.NONE, movement).step(world, state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (result.onGround()) return null;
            state = new PhysicsState(result.position(), result.velocity(), false,
                    result.fallDistance(), PlayerPose.GLIDING);
        }
        return state;
    }

    /** Fixed unpowered reserve after an observed rejection: the same reserve-plus-bridge
     * horizon an unpowered lease must prove, airborne, loaded and hazard-free. */
    private static List<FlightControlAdmission.Step> rejectionHandOver(World world,
            MovementParameters movement, PhysicsState observed, float yaw,
            WaypointFlightTarget corridor, boolean preserveFallMargin, Runnable validateRequest) {
        int ticks = RESERVE_TICKS + HANDOFF_BRIDGE_TICKS;
        for (float pitch : new float[] {-45, 0}) {
            var control = new ElytraFlightController.Decision(yaw, pitch, false,
                    ElytraFlightController.Phase.CRUISE);
            var steps = new ArrayList<FlightControlAdmission.Step>(ticks);
            PhysicsState state = observed;
            for (int tick = 0; tick < ticks; tick++) {
                validateRequest.run();
                var result = new GlidingPhysics(control.yaw(), control.pitch(), false,
                        PlayerInput.NONE, movement).step(world, state,
                                PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                if (!safeAirborneTransition(world, movement, state, control, false, result,
                        preserveFallMargin, corridor)) break;
                state = new PhysicsState(result.position(), result.velocity(), false,
                        result.fallDistance(), PlayerPose.GLIDING);
                steps.add(new FlightControlAdmission.Step(control, state));
            }
            if (steps.size() == ticks) return List.copyOf(steps);
        }
        return List.of();
    }

    private static PreparedEpisode rocketEpisode(World world, MovementParameters movement,
            PhysicsState initial, List<ElytraFlightController.Decision> controls,
            FireworkSchedule.LifetimeBounds lifetime) {
        int executableTicks = lifetime.maximumPoweredTicks() + 1;
        int prefixEnd = executableTicks - RESERVE_TICKS;
        var steps = rocketNominalSteps(world, movement, initial, controls, lifetime.maximumPoweredTicks());
        var prefix = withFirstRocket(steps.subList(0, prefixEnd), true);
        var reserve = List.copyOf(steps.subList(prefixEnd, executableTicks));
        var bridge = List.copyOf(steps.subList(executableTicks, steps.size()));
        return new PreparedEpisode(prefix, reserve, bridge, bridge.get(0).expected());
    }

    /** Both physical outcomes need representation in the same fixed beam. Ranking
     * only powered progress can discard every control that terminates rejection. */
    private static List<RocketSearchNode> diverseRocketNodes(List<RocketSearchNode> sorted,
            boolean rejectionMustStayAirborne) {
        if (sorted.size() <= ROCKET_SEARCH_BEAM_WIDTH) return List.copyOf(sorted);
        var selected = new ArrayList<>(sorted.subList(0, ROCKET_SEARCH_BEAM_WIDTH - 1));
        var rejection = sorted.stream().min(java.util.Comparator
                .comparingDouble((RocketSearchNode node) -> {
                    var branch = node.branches().get(0);
                    double projectedAltitude = branch.state().position().y + branch.state().velocity().y * 3;
                    if (rejectionMustStayAirborne) return -projectedAltitude;
                    if (branch.landed()) return Double.NEGATIVE_INFINITY;
                    var velocity = branch.state().velocity();
                    double horizontalSpeedSquared = velocity.x * velocity.x + velocity.z * velocity.z;
                    // Rejection needs a low-energy landing, not merely the fastest dive.
                    // A weak powered-progress term keeps its representative useful when
                    // thrust does arrive; full branch proof still decides admissibility.
                    return projectedAltitude + 30 * horizontalSpeedSquared + .01 * node.score();
                }).thenComparingDouble(RocketSearchNode::score)).orElseThrow();
        selected.add(selected.contains(rejection) ? sorted.get(ROCKET_SEARCH_BEAM_WIDTH - 1)
                : rejection);
        return List.copyOf(selected);
    }

    private static List<RocketBranch> rocketSearchBranches(PhysicsState initial,
            FireworkSchedule.LifetimeBounds lifetime) {
        return rocketSearchBranches(initial, lifetime, false);
    }

    /** A hand-over treats absence at the observation tick as rejection, so every
     * acknowledgement before that tick, not only immediate or next-tick, is a branch. */
    private static List<RocketBranch> rocketSearchBranches(PhysicsState initial,
            FireworkSchedule.LifetimeBounds lifetime, boolean rejectionHandOver) {
        int maximumAcknowledgementDelay = rejectionHandOver
                ? FlightControlAdmission.REJECTED_ROCKET_OBSERVATION_TICKS - 1 : 1;
        var branches = new ArrayList<RocketBranch>();
        branches.add(new RocketBranch(0, 0, initial, false));
        for (int poweredTicks = lifetime.minimumPoweredTicks();
                poweredTicks <= lifetime.maximumPoweredTicks(); poweredTicks++) {
            for (int delay = 0; delay <= maximumAcknowledgementDelay; delay++)
                branches.add(new RocketBranch(poweredTicks, delay, initial, false));
        }
        return List.copyOf(branches);
    }

    private static boolean validRocketControls(World world, MovementParameters movement,
            PhysicsState initial, List<ElytraFlightController.Decision> controls,
            FireworkSchedule.LifetimeBounds lifetime, WaypointFlightTarget corridor,
            boolean preserveFallMargin, Runnable validateRequest, Float handOverYaw) {
        if (!validRejectedRocketControls(world, movement, initial, controls,
                controls.size() - HANDOFF_BRIDGE_TICKS, corridor, preserveFallMargin, validateRequest,
                handOverYaw)) return false;
        List<RocketBranch> replay = rocketSearchBranches(initial, lifetime, handOverYaw != null).stream()
                .filter(branch -> branch.lifetime() > 0).toList();
        for (int tick = 0; tick < controls.size(); tick++) {
            validateRequest.run();
            replay = advanceRocketBranches(world, movement, replay, controls.get(tick), tick,
                    corridor, preserveFallMargin, validateRequest);
            if (replay == null) return false;
        }
        return true;
    }

    /** A rejected request may leave the powered route only when this same fixed-control
     * episode closes on loaded, damage-free support. Otherwise it remains route-bound,
     * or, with a hand-over heading, reaches the observation tick inside the route with
     * a proved unpowered reserve. */
    private static boolean validRejectedRocketControls(World world, MovementParameters movement,
            PhysicsState initial, List<ElytraFlightController.Decision> controls,
            int executableTicks, WaypointFlightTarget corridor, boolean preserveFallMargin, Runnable validateRequest,
            Float handOverYaw) {
        PhysicsState state = initial;
        boolean outsideCorridor = false;
        for (int tick = 0; tick < controls.size(); tick++) {
            validateRequest.run();
            if (handOverYaw != null && tick == FlightControlAdmission.REJECTED_ROCKET_OBSERVATION_TICKS
                    && !outsideCorridor && !rejectionHandOver(world, movement, state, handOverYaw,
                            corridor, preserveFallMargin, validateRequest).isEmpty()) return true;
            var control = controls.get(tick);
            var result = new GlidingPhysics(control.yaw(), control.pitch(), false,
                    PlayerInput.NONE, movement).step(world, state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            outsideCorridor |= corridor != null
                    && !corridor.contains(state.position(), result.position());
            if (result.onGround()) {
                if ((!outsideCorridor || tick < executableTicks)
                        && safeRejectedRocketLanding(world, movement, state, result)) return true;
                return false;
            }
            if (!safeAirborneTransition(world, movement, state, control, false, result,
                    preserveFallMargin, null)) {
                return false;
            }
            state = new PhysicsState(result.position(), result.velocity(), false,
                    result.fallDistance(), PlayerPose.GLIDING);
            if (outsideCorridor && tick + 1 >= executableTicks) return false;
        }
        return !outsideCorridor;
    }

    /** Candidate generation only. Full swept-body, hazard and corridor proof is
     * replayed by validRocketControls before any episode can be published. */
    private static List<RocketBranch> advanceRocketSearchBranches(World world,
            MovementParameters movement, List<RocketBranch> branches,
            ElytraFlightController.Decision control, int tick, WaypointFlightTarget corridor,
            boolean preserveFallMargin, Runnable validateRequest, Float handOverYaw) {
        var advanced = new ArrayList<RocketBranch>(branches.size());
        var transitions = new java.util.HashMap<RocketSearchTransition, PhysicsState>();
        var checkedPoweredStates = new java.util.HashSet<PhysicsState>();
        var waypoints = corridor == null ? List.<Vector3d>of() : corridor.waypoints();
        for (var branch : branches) {
            validateRequest.run();
            if (branch.landed()) {
                advanced.add(branch);
                continue;
            }
            // An observed rejection with a proved hand-over is closed, like a safe landing.
            if (handOverYaw != null && branch.lifetime() == 0
                    && tick == FlightControlAdmission.REJECTED_ROCKET_OBSERVATION_TICKS
                    && !rejectionHandOver(world, movement, branch.state(), handOverYaw, corridor,
                            preserveFallMargin, validateRequest).isEmpty()) {
                advanced.add(new RocketBranch(0, 0, branch.state(), true));
                continue;
            }
            boolean powered = tick >= branch.acknowledgementDelay() && tick < branch.lifetime();
            var key = new RocketSearchTransition(branch.state(), powered);
            PhysicsState next = transitions.get(key);
            if (next == null) {
                var result = new GlidingPhysics(control.yaw(), control.pitch(), powered,
                        PlayerInput.NONE, movement).step(world, branch.state(),
                                PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                if (!result.position().isFinite() || !result.velocity().isFinite()
                        || result.collidedX() || result.collidedZ()
                        || preserveFallMargin && !(result.fallDistance() < movement.safeFallDistance())
                        || powered && result.onGround()) {
                    return null;
                }
                if (result.onGround() && !safeRejectedRocketLanding(world, movement,
                        branch.state(), result)) {
                    return null;
                }
                if (branch.state().velocity().y > .003 && !result.onGround()) {
                    var travel = powered ? new GlidingPhysics(control.yaw(), control.pitch(), false,
                            PlayerInput.NONE, movement).step(world, branch.state(),
                                    PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height()) : result;
                    if (travel.velocity().y == 0) return null;
                }
                next = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                        result.fallDistance(), PlayerPose.GLIDING);
                transitions.put(key, next);
            }
            // A powered lifetime remains airborne and route-bound even after expiry.
            // Reject geometrically impossible candidates before they consume the beam;
            // full swept-body corridor validation still runs on every final branch.
            if (branch.lifetime() > 0 && checkedPoweredStates.add(next) && (next.onGround()
                    || corridor != null && !corridor.executionWitness()
                    && distanceToRoute(corridor, waypoints, next.position()) > corridor.corridorRadius()))
                return null;
            advanced.add(new RocketBranch(branch.lifetime(), branch.acknowledgementDelay(),
                    next, next.onGround()));
        }
        return List.copyOf(advanced);
    }

    private static List<RocketBranch> advanceRocketBranches(World world,
            MovementParameters movement, List<RocketBranch> branches,
            ElytraFlightController.Decision control, int tick, WaypointFlightTarget corridor,
            boolean preserveFallMargin, Runnable validateRequest) {
        var advanced = new ArrayList<RocketBranch>(branches.size());
        var transitions = new java.util.HashMap<RocketSearchTransition, PhysicsState>();
        for (var branch : branches) {
            validateRequest.run();
            if (branch.landed()) {
                advanced.add(branch);
                continue;
            }
            boolean powered = tick >= branch.acknowledgementDelay() && tick < branch.lifetime();
            var key = new RocketSearchTransition(branch.state(), powered);
            PhysicsState next = transitions.get(key);
            if (next == null) {
                var result = new GlidingPhysics(control.yaw(), control.pitch(), powered,
                        PlayerInput.NONE, movement).step(world, branch.state(),
                                PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                if (!safeAirborneTransition(world, movement, branch.state(), control, powered, result,
                        preserveFallMargin, corridor)) return null;
                next = new PhysicsState(result.position(), result.velocity(), false,
                        result.fallDistance(), PlayerPose.GLIDING);
                transitions.put(key, next);
            }
            advanced.add(new RocketBranch(branch.lifetime(), branch.acknowledgementDelay(), next, false));
        }
        return List.copyOf(advanced);
    }

    private static List<ElytraFlightController.Decision> rocketControlBank(
            ElytraFlightController.Decision selected, ElytraFlightController.Decision previous) {
        var controls = new ArrayList<ElytraFlightController.Decision>();
        float[] yaws = {selected.yaw(), selected.yaw() + 180, selected.yaw() - 45,
                selected.yaw() + 45, previous.yaw()};
        float[] pitches = {selected.pitch(), 0, POWERED_CRUISE_RESERVE_PITCH, -55, 20, 60};
        for (float yaw : yaws) for (float pitch : pitches) {
            var control = new ElytraFlightController.Decision(yaw, pitch, false, selected.phase());
            if (!controls.contains(control)) controls.add(control);
        }
        return List.copyOf(controls);
    }

    private static double rocketSearchScore(List<RocketBranch> branches, Vector3d goal,
            ElytraFlightController.Decision control, WaypointFlightTarget corridor,
            LocalLookaheadSolver.MovingTarget tracking, int elapsedTicks) {
        // Geometry owns clearance, not a moving target's stopping position. Every lifetime
        // and acknowledgement branch shares the source clock, including its unpowered tail.
        Vector3d pursuitGoal = tracking == null ? goal : tracking.predict(elapsedTicks);
        Vector3d pursuitVelocity = tracking == null ? new Vector3d() : tracking.velocity();
        double score = 0;
        var scoredStates = new java.util.HashSet<PhysicsState>();
        var waypoints = corridor == null ? List.<Vector3d>of() : corridor.waypoints();
        for (var branch : branches) {
            if (branch.landed() || branch.lifetime() == 0 || !scoredStates.add(branch.state())) continue;
            Vector3d position = branch.state().position();
            Vector3d velocity = branch.state().velocity();
            double distance = Math.hypot(position.x - pursuitGoal.x, position.z - pursuitGoal.z);
            double radialX = distance == 0 ? 0 : (position.x - pursuitGoal.x) / distance;
            double radialZ = distance == 0 ? 0 : (position.z - pursuitGoal.z) / distance;
            double relativeVelocityX = velocity.x - pursuitVelocity.x;
            double relativeVelocityZ = velocity.z - pursuitVelocity.z;
            double outward = relativeVelocityX * radialX + relativeVelocityZ * radialZ;
            double verticalOutward = tracking == null ? 0 : Math.signum(position.y - pursuitGoal.y)
                    * (velocity.y - pursuitVelocity.y);
            double horizontalSpeed = Math.hypot(relativeVelocityX, relativeVelocityZ);
            double corridorError = corridor == null ? 0
                    : distanceToRoute(corridor, waypoints, new Vector3d(position).fma(3, velocity));
            if (corridor != null && (tracking != null || goal.y > corridor.goal().y))
                corridorError = Math.max(0, corridorError - corridor.corridorRadius());
            double projectedAltitude = position.y + velocity.y * 3;
            double clearanceAltitude = tracking != null && corridor != null
                    ? routeAltitudeAt(corridor, waypoints, new Vector3d(position).fma(3, velocity)) : goal.y;
            double altitudeError = corridor != null && !corridor.terrainValidated() ? 0
                    : projectedAltitude < clearanceAltitude
                            ? (clearanceAltitude - projectedAltitude) * 12 : projectedAltitude - clearanceAltitude;
            score = Math.max(score, distance + Math.max(0, outward) * 8
                    + Math.max(0, verticalOutward) * 8 + horizontalSpeed * 4 + Math.abs(velocity.y) * 4
                    + corridorError * 20 + altitudeError);
        }
        return score + Math.abs(control.pitch()) * .001;
    }

    private static double routeAltitudeAt(WaypointFlightTarget route, List<Vector3d> waypoints,
            Vector3d position) {
        double nearestDistance = Double.POSITIVE_INFINITY;
        double altitude = route.position().y;
        for (int index = Math.max(1, route.index() - 1); index < waypoints.size(); index++) {
            Vector3d start = waypoints.get(index - 1);
            Vector3d end = waypoints.get(index);
            double deltaX = end.x - start.x;
            double deltaZ = end.z - start.z;
            double lengthSquared = deltaX * deltaX + deltaZ * deltaZ;
            double fraction = lengthSquared == 0 ? 0 : Math.max(0, Math.min(1,
                    ((position.x - start.x) * deltaX + (position.z - start.z) * deltaZ) / lengthSquared));
            double distance = Math.hypot(position.x - start.x - fraction * deltaX,
                    position.z - start.z - fraction * deltaZ);
            if (distance >= nearestDistance) continue;
            nearestDistance = distance;
            // Clear the top of an ascending edge before crossing it. Horizontal
            // projection alone maps a vertical climb to its foot, below the obstacle.
            altitude = end.y > start.y ? end.y : start.y + fraction * (end.y - start.y);
        }
        return altitude;
    }

    private static double distanceToRoute(WaypointFlightTarget route, List<Vector3d> waypoints,
            Vector3d position) {
        if (waypoints.size() == 1) return position.distance(waypoints.get(0));
        double distance = Double.POSITIVE_INFINITY;
        for (int index = Math.max(1, route.index() - 1); index < waypoints.size(); index++)
            distance = Math.min(distance, distanceToSegment(position,
                    waypoints.get(index - 1), waypoints.get(index)));
        return distance;
    }

    private static double distanceToSegment(Vector3d position, Vector3d start, Vector3d end) {
        Vector3d edge = new Vector3d(end).sub(start);
        double fraction = edge.lengthSquared() == 0 ? 0 : Math.max(0, Math.min(1,
                new Vector3d(position).sub(start).dot(edge) / edge.lengthSquared()));
        return position.distance(new Vector3d(start).fma(fraction, edge));
    }

    private static List<FlightControlAdmission.Step> rocketNominalSteps(World world,
            MovementParameters movement, PhysicsState initial,
            List<ElytraFlightController.Decision> controls, int maximumLifetime) {
        var steps = new ArrayList<FlightControlAdmission.Step>(controls.size());
        PhysicsState state = initial;
        for (int tick = 0; tick < controls.size(); tick++) {
            var control = controls.get(tick);
            steps.add(new FlightControlAdmission.Step(control, state));
            var result = new GlidingPhysics(control.yaw(), control.pitch(), tick < maximumLifetime,
                    PlayerInput.NONE, movement).step(world, state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                    result.fallDistance(), PlayerPose.GLIDING);
        }
        return List.copyOf(steps);
    }


    record DangerousEntry(PhysicsState state, int elapsedTicks, boolean exactTick) {
        DangerousEntry(PhysicsState state, int elapsedTicks) { this(state, elapsedTicks, true); }
    }
    record ExecutionBranch(List<PhysicsState> states) {
        ExecutionBranch { states = List.copyOf(states); }
    }
    record AttachmentExpiryProof(WaypointFlightTarget corridor,
                                 List<ExecutionBranch> branches) {
        AttachmentExpiryProof { branches = List.copyOf(branches); }
    }
    record RocketTransitionProof(WaypointFlightTarget corridor,
                                 List<ExecutionBranch> branches) {
        RocketTransitionProof { branches = List.copyOf(branches); }
    }

    /** Lightweight candidate gate used during ranking; witness construction happens only
     * for the selected prefix and its longest provable reserve.
     */
    static int attachmentExpiryTicks(World world, MovementParameters movement,
            PhysicsState initial, List<FlightControlAdmission.Step> prefix,
            boolean preserveFallMargin, WaypointFlightTarget corridor) {
        int safeTicks = prefix.size();
        for (int transitionTick = 0; transitionTick <= prefix.size(); transitionTick++) {
            PhysicsState state = initial;
            for (int tick = 0; tick < safeTicks; tick++) {
                var control = prefix.get(tick).control();
                boolean powered = tick < transitionTick;
                var result = new GlidingPhysics(control.yaw(), control.pitch(), powered,
                        PlayerInput.NONE, movement).step(world, state,
                                PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                if (!safeAirborneTransition(world, movement, state, control, powered, result,
                        preserveFallMargin, corridor)) {
                    safeTicks = tick;
                    break;
                }
                state = new PhysicsState(result.position(), result.velocity(), false,
                        result.fallDistance(), PlayerPose.GLIDING);
            }
            if (safeTicks == 0) return 0;
        }
        return safeTicks;
    }

    /** A current attachment may expire on any tick. Every possible branch must remain
     * loaded, collision/damage-free and airborne before its path becomes a witness.
     */
    static AttachmentExpiryProof attachmentExpiryProof(World world,
            MovementParameters movement, PhysicsState initial,
            List<FlightControlAdmission.Step> prefix,
            List<FlightControlAdmission.Step> reserve, Vector3d goal,
            boolean preserveFallMargin, WaypointFlightTarget corridor) {
        return attachmentExpiryProof(world, movement, initial, prefix, reserve, List.of(), goal,
                preserveFallMargin, corridor);
    }

    static AttachmentExpiryProof attachmentExpiryProof(World world,
            MovementParameters movement, PhysicsState initial,
            List<FlightControlAdmission.Step> prefix,
            List<FlightControlAdmission.Step> reserve,
            List<FlightControlAdmission.Step> bridge, Vector3d goal,
            boolean preserveFallMargin, WaypointFlightTarget corridor) {
        return attachmentExpiryProof(world, movement, initial, prefix, reserve, bridge, goal,
                preserveFallMargin, corridor, NO_CANCELLATION);
    }

    private static AttachmentExpiryProof attachmentExpiryProof(World world,
            MovementParameters movement, PhysicsState initial,
            List<FlightControlAdmission.Step> prefix,
            List<FlightControlAdmission.Step> reserve,
            List<FlightControlAdmission.Step> bridge, Vector3d goal,
            boolean preserveFallMargin, WaypointFlightTarget corridor, Runnable validateRequest) {
        validateRequest.run();
        var controls = new ArrayList<ElytraFlightController.Decision>(
                prefix.size() + reserve.size() + bridge.size());
        prefix.forEach(step -> controls.add(step.control()));
        reserve.forEach(step -> controls.add(step.control()));
        bridge.forEach(step -> controls.add(step.control()));
        var paths = new ArrayList<List<Vector3d>>(controls.size() + 1);
        var branches = new ArrayList<ExecutionBranch>(controls.size() + 1);
        for (int transitionTick = 0; transitionTick <= controls.size(); transitionTick++) {
            validateRequest.run();
            PhysicsState state = initial;
            var states = new ArrayList<PhysicsState>(controls.size() + 1);
            var path = new ArrayList<Vector3d>(controls.size() + 1);
            states.add(state);
            path.add(state.position());
            for (int tick = 0; tick < controls.size(); tick++) {
                validateRequest.run();
                var control = controls.get(tick);
                boolean powered = tick < transitionTick;
                var result = new GlidingPhysics(control.yaw(), control.pitch(), powered,
                        PlayerInput.NONE, movement).step(world, state,
                                PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                if (!safeAirborneTransition(world, movement, state, control, powered, result,
                        preserveFallMargin, corridor)) return null;
                state = new PhysicsState(result.position(), result.velocity(), false,
                        result.fallDistance(), PlayerPose.GLIDING);
                states.add(state);
                path.add(state.position());
            }
            paths.add(List.copyOf(path));
            branches.add(new ExecutionBranch(states));
        }
        return new AttachmentExpiryProof(WaypointFlightTarget.executionWitness(paths, goal, false), branches);
    }

    /** The request tick starts vanilla's lifetime; acknowledgement only reveals remaining thrust.
     * Physics therefore has one rejected/no-thrust branch and one request-relative branch for
     * every legal lifetime. Powered branches remain loaded, hazard-free, airborne and route-bound.
     * Rejection may leave the route only when executable controls reach safe support; proof-only
     * handoff cannot supply that landing control, nor can contact truncate possible thrust.
     */
    static RocketTransitionProof rocketTransitionProof(World world,
            MovementParameters movement, PhysicsState initial,
            List<FlightControlAdmission.Step> prefix,
            List<FlightControlAdmission.Step> reserve, Vector3d goal,
            HazardRecovery.Context resources, WaypointFlightTarget corridor) {
        return rocketTransitionProof(world, movement, initial, prefix, reserve, List.of(), goal,
                resources, corridor);
    }

    static RocketTransitionProof rocketTransitionProof(World world,
            MovementParameters movement, PhysicsState initial,
            List<FlightControlAdmission.Step> prefix,
            List<FlightControlAdmission.Step> reserve,
            List<FlightControlAdmission.Step> bridge, Vector3d goal,
            HazardRecovery.Context resources, WaypointFlightTarget corridor) {
        return rocketTransitionProof(world, movement, initial, prefix, reserve, bridge, goal,
                resources, corridor, false);
    }

    static RocketTransitionProof rocketTransitionProof(World world,
            MovementParameters movement, PhysicsState initial,
            List<FlightControlAdmission.Step> prefix,
            List<FlightControlAdmission.Step> reserve,
            List<FlightControlAdmission.Step> bridge, Vector3d goal,
            HazardRecovery.Context resources, WaypointFlightTarget corridor, boolean preserveFallMargin) {
        return rocketTransitionProof(world, movement, initial, prefix, reserve, bridge, goal,
                resources, corridor, preserveFallMargin, NO_CANCELLATION);
    }

    static RocketTransitionProof rocketTransitionProof(World world,
            MovementParameters movement, PhysicsState initial,
            List<FlightControlAdmission.Step> prefix,
            List<FlightControlAdmission.Step> reserve,
            List<FlightControlAdmission.Step> bridge, Vector3d goal,
            HazardRecovery.Context resources, WaypointFlightTarget corridor, boolean preserveFallMargin,
            List<FlightControlAdmission.Step> rejectionHandOver) {
        return rocketTransitionProof(world, movement, initial, prefix, reserve, bridge, goal,
                resources, corridor, preserveFallMargin, NO_CANCELLATION, rejectionHandOver);
    }

    private static RocketTransitionProof rocketTransitionProof(World world,
            MovementParameters movement, PhysicsState initial,
            List<FlightControlAdmission.Step> prefix,
            List<FlightControlAdmission.Step> reserve,
            List<FlightControlAdmission.Step> bridge, Vector3d goal,
            HazardRecovery.Context resources, WaypointFlightTarget corridor, boolean preserveFallMargin, Runnable validateRequest) {
        return rocketTransitionProof(world, movement, initial, prefix, reserve, bridge, goal,
                resources, corridor, preserveFallMargin, validateRequest, List.of());
    }

    /** With a hand-over, admission keeps the rejected rocket controls only while they
     * stay safe; otherwise it switches at the observation tick. Each executed rejected
     * path is a witness branch, and at least one of them must exist. */
    private static RocketTransitionProof rocketTransitionProof(World world,
            MovementParameters movement, PhysicsState initial,
            List<FlightControlAdmission.Step> prefix,
            List<FlightControlAdmission.Step> reserve,
            List<FlightControlAdmission.Step> bridge, Vector3d goal,
            HazardRecovery.Context resources, WaypointFlightTarget corridor, boolean preserveFallMargin,
            Runnable validateRequest, List<FlightControlAdmission.Step> rejectionHandOver) {
        validateRequest.run();
        int[] flights = resources.rocketFlightDurations();
        if (resources.attached() || resources.cooldownTicks() > 0 || flights.length == 0) return null;
        var controls = new ArrayList<ElytraFlightController.Decision>(
                prefix.size() + reserve.size() + bridge.size());
        prefix.forEach(step -> controls.add(step.control()));
        reserve.forEach(step -> controls.add(step.control()));
        bridge.forEach(step -> controls.add(step.control()));
        int minimumLifetime = Math.toIntExact(FireworkSchedule.minimumBoostTicks(flights[0]));
        int maximumLifetime = Math.toIntExact(FireworkSchedule.maximumBoostTicks(flights[0]));
        var paths = new ArrayList<List<Vector3d>>();
        var branches = new ArrayList<ExecutionBranch>();
        var rejectedStates = strictRejectedStates(world, movement, initial, controls,
                prefix.size() + reserve.size(), corridor, preserveFallMargin, validateRequest);
        var handOverStates = rejectionHandOver.isEmpty() ? null : handOverRejectedStates(world, movement,
                initial, controls, rejectionHandOver, corridor, preserveFallMargin, validateRequest);
        if (rejectedStates == null && handOverStates == null) return null;
        for (var states : java.util.Arrays.asList(rejectedStates, handOverStates)) {
            if (states == null) continue;
            paths.add(states.stream().map(PhysicsState::position).toList());
            branches.add(new ExecutionBranch(states));
        }
        // Immediate and next-tick observation share request-relative expiry. Replay
        // equal physical transitions once per tick; retain every branch's full witness.
        List<RocketBranch> poweredBranches = rocketSearchBranches(initial,
                new FireworkSchedule.LifetimeBounds(minimumLifetime, maximumLifetime),
                !rejectionHandOver.isEmpty()).stream()
                .filter(branch -> branch.lifetime() > 0).toList();
        var poweredStates = new ArrayList<List<PhysicsState>>(poweredBranches.size());
        for (var branch : poweredBranches) {
            var states = new ArrayList<PhysicsState>(controls.size() + 1);
            states.add(branch.state());
            poweredStates.add(states);
        }
        for (int tick = 0; tick < controls.size(); tick++) {
            validateRequest.run();
            poweredBranches = advanceRocketBranches(world, movement, poweredBranches,
                    controls.get(tick), tick, corridor, preserveFallMargin, validateRequest);
            if (poweredBranches == null) return null;
            for (int branch = 0; branch < poweredBranches.size(); branch++)
                poweredStates.get(branch).add(poweredBranches.get(branch).state());
        }
        for (var states : poweredStates) {
            paths.add(states.stream().map(PhysicsState::position).toList());
            branches.add(new ExecutionBranch(states));
        }
        WaypointFlightTarget proofCorridor = corridor == null
                ? WaypointFlightTarget.executionWitness(paths, goal, false)
                : corridor;
        return new RocketTransitionProof(proofCorridor, branches);
    }


    /** Rejected request on the rocket controls themselves: route-bound, or closing on
     * safe support within the executable ticks. Null when that path is not proved. */
    private static List<PhysicsState> strictRejectedStates(World world, MovementParameters movement,
            PhysicsState initial, List<ElytraFlightController.Decision> controls, int executableTicks,
            WaypointFlightTarget corridor, boolean preserveFallMargin, Runnable validateRequest) {
        PhysicsState rejectedState = initial;
        var rejectedStates = new ArrayList<PhysicsState>(controls.size() + 1);
        rejectedStates.add(rejectedState);
        boolean outsideCorridor = false;
        for (int tick = 0; tick < controls.size(); tick++) {
            validateRequest.run();
            var control = controls.get(tick);
            var result = new GlidingPhysics(control.yaw(), control.pitch(), false,
                    PlayerInput.NONE, movement).step(world, rejectedState,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            outsideCorridor |= corridor != null
                    && !corridor.contains(rejectedState.position(), result.position());
            if (result.onGround()) {
                if (outsideCorridor && tick >= executableTicks
                        || !safeRejectedRocketLanding(world, movement, rejectedState, result)) return null;
            } else if (!safeAirborneTransition(world, movement, rejectedState, control, false,
                    result, preserveFallMargin, null)) return null;
            rejectedState = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                    result.fallDistance(), PlayerPose.GLIDING);
            rejectedStates.add(rejectedState);
            if (rejectedState.onGround()) break;
            if (outsideCorridor && tick + 1 >= executableTicks) return null;
        }
        if (outsideCorridor && !rejectedState.onGround()) return null;
        return rejectedStates;
    }

    /** Rejected request up to the observation tick, then the fixed unpowered hand-over. */
    private static List<PhysicsState> handOverRejectedStates(World world, MovementParameters movement,
            PhysicsState initial, List<ElytraFlightController.Decision> controls,
            List<FlightControlAdmission.Step> handOver, WaypointFlightTarget corridor,
            boolean preserveFallMargin, Runnable validateRequest) {
        int observed = FlightControlAdmission.REJECTED_ROCKET_OBSERVATION_TICKS;
        if (controls.size() < observed) return null;
        var replay = new ArrayList<>(controls.subList(0, observed));
        handOver.forEach(step -> replay.add(step.control()));
        PhysicsState state = initial;
        var states = new ArrayList<PhysicsState>(replay.size() + 1);
        states.add(state);
        for (var control : replay) {
            validateRequest.run();
            var result = new GlidingPhysics(control.yaw(), control.pitch(), false,
                    PlayerInput.NONE, movement).step(world, state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (!safeAirborneTransition(world, movement, state, control, false, result,
                    preserveFallMargin, corridor)) return null;
            state = new PhysicsState(result.position(), result.velocity(), false,
                    result.fallDistance(), PlayerPose.GLIDING);
            states.add(state);
        }
        return states;
    }

    private static boolean safeRejectedRocketLanding(World world, MovementParameters movement,
            PhysicsState state, PhysicsResult result) {
        Vector3d position = result.position();
        double landingFallDistance = state.fallDistance()
                + Math.max(0, state.position().y - position.y);
        return position.isFinite() && result.velocity().isFinite()
                && Double.isFinite(result.fallDistance())
                && position.y >= World.getMinWorldY()
                && position.y + PlayerPose.GLIDING.height() <= World.getMaxWorldY()
                && !result.collidedX() && !result.collidedZ()
                && landingFallDistance < movement.safeFallDistance()
                && LocalAirRoute.visible(world, state.position(), position, PlayerPose.GLIDING)
                && hazardFree(world, state, result)
                && FlightHazards.safeLanding(world, position);
    }

    private static boolean safeAirborneTransition(World world, MovementParameters movement,
            PhysicsState state, ElytraFlightController.Decision control, boolean powered,
            PhysicsResult result, boolean preserveFallMargin, WaypointFlightTarget corridor) {
        Vector3d position = result.position();
        var travel = powered ? new GlidingPhysics(control.yaw(), control.pitch(), false,
                PlayerInput.NONE, movement).step(world, state,
                        PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height()) : result;
        return position.isFinite() && result.velocity().isFinite() && Double.isFinite(result.fallDistance())
                && position.y >= World.getMinWorldY()
                && position.y + PlayerPose.GLIDING.height() <= World.getMaxWorldY()
                && !result.onGround() && !result.collidedX() && !result.collidedZ()
                && (!preserveFallMargin || result.fallDistance() < movement.safeFallDistance())
                && (state.velocity().y <= .003 || travel.velocity().y != 0)
                && LocalAirRoute.visible(world, state.position(), position, PlayerPose.GLIDING)
                && (corridor == null || corridor.contains(state.position(), position))
                && hazardFree(world, state, result);
    }

    static DangerousEntry firstDangerousEntry(World world, List<PhysicsState> states) {
        return new DangerousEntryScan(world).firstEntry(states);
    }

    private record ExposureSweep(PhysicsState before, PhysicsState after) {}

    /** Lifetime branches share long identical paths. Reuse only this proof's terrain
     * observations; no result survives the worker invocation or skips an entry gate. */
    private static final class DangerousEntryScan {
        private final World world;
        private final java.util.Map<Vector3d, Boolean> exposedPositions = new java.util.HashMap<>();
        private final java.util.Map<ExposureSweep, Boolean> exposedSweeps = new java.util.HashMap<>();

        private DangerousEntryScan(World world) { this.world = world; }

        private boolean exposed(PhysicsState state) {
            return exposedPositions.computeIfAbsent(state.position(),
                    position -> DangerousEntryGate.exposed(world, state));
        }

        private DangerousEntry firstEntry(List<PhysicsState> states) {
            if (states.isEmpty() || exposed(states.get(0))) return null;
            for (int tick = 1; tick < states.size(); tick++) {
                PhysicsState before = states.get(tick - 1);
                PhysicsState state = states.get(tick);
                if (exposed(state)) return new DangerousEntry(state, tick);
                // Fractional resource checkpoints cannot certify remaining inventory.
                if (exposedSweeps.computeIfAbsent(new ExposureSweep(before, state),
                        sweep -> exposedBetween(sweep.before(), sweep.after(), this::exposed)))
                    return new DangerousEntry(state, tick, false);
            }
            return null;
        }
    }

    static DangerousEntry firstDangerousEntry(World world, MovementParameters movement,
            PhysicsState initial, List<FlightControlAdmission.Step> prefix,
            List<FlightControlAdmission.Step> reserve, PhysicsState episodeEnd) {
        var states = new ArrayList<PhysicsState>(prefix.size() + reserve.size() + 1);
        PhysicsState state = initial;
        states.add(state);
        for (var step : prefix) {
            if (!step.expected().equals(state)) return new DangerousEntry(state, 0, false);
            state = nextState(world, movement, state, step.control());
            states.add(state);
        }
        for (var step : reserve) {
            if (!step.expected().equals(state)) return new DangerousEntry(state, 0, false);
            state = nextState(world, movement, state, step.control());
            states.add(state);
        }
        if (!state.equals(episodeEnd)) return new DangerousEntry(state, 0, false);
        return firstDangerousEntry(world, states);
    }

    private static PhysicsState nextState(World world, MovementParameters movement,
            PhysicsState state, ElytraFlightController.Decision control) {
        var result = new GlidingPhysics(control.yaw(), control.pitch(), false,
                PlayerInput.NONE, movement).step(world, state,
                        PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        return new PhysicsState(result.position(), result.velocity(), result.onGround(),
                result.fallDistance(), PlayerPose.GLIDING);
    }

    private static boolean exposedBetween(PhysicsState before, PhysicsState after,
            java.util.function.Predicate<PhysicsState> exposed) {
        Vector3d from = before.position(), to = after.position();
        var events = new java.util.TreeSet<Double>();
        events.add(0d);
        events.add(1d);
        int[] remaining = {128};
        double radius = PlayerPose.HALF_WIDTH;
        if (!addGridCrossings(events, from.x - radius, to.x - radius, remaining)
                || !addGridCrossings(events, from.x + radius, to.x + radius, remaining)
                || !addGridCrossings(events, from.z - radius, to.z - radius, remaining)
                || !addGridCrossings(events, from.z + radius, to.z + radius, remaining)
                || !addGridCrossings(events, from.y, to.y, remaining)) return true;
        Double previous = null;
        for (double event : events) {
            if (event > 0 && event < 1 && exposed.test(
                    interpolationState(before, after, event))) return true;
            if (previous != null && event > previous) {
                double middle = (previous + event) * .5;
                if (middle > 0 && middle < 1 && exposed.test(
                        interpolationState(before, after, middle))) return true;
            }
            previous = event;
        }
        return false;
    }

    private static boolean addGridCrossings(java.util.Set<Double> events,
            double from, double to, int[] remaining) {
        double delta = to - from;
        if (delta == 0) return true;
        int first = (int) Math.ceil(Math.min(from, to));
        int last = (int) Math.floor(Math.max(from, to));
        for (int boundary = first; boundary <= last; boundary++) {
            double fraction = (boundary - from) / delta;
            if (fraction <= 0 || fraction >= 1) continue;
            if (--remaining[0] < 0) return false;
            events.add(fraction);
        }
        return true;
    }

    private static PhysicsState interpolationState(PhysicsState before,
            PhysicsState after, double fraction) {
        return new PhysicsState(before.position().lerp(after.position(), fraction),
                before.velocity().lerp(after.velocity(), fraction), false,
                before.fallDistance() + (after.fallDistance() - before.fallDistance()) * fraction,
                PlayerPose.GLIDING);
    }

    private static boolean permitsDangerousEntry(World world, PhysicsState initial,
            DangerousEntry entry, MovementParameters movement, HazardRecovery.Context resources,
            boolean allowExposedProgress, boolean knownReturn, boolean rocket) {
        if (entry == null) return true;
        if (!entry.exactTick() || !allowExposedProgress) return false;
        HazardRecovery.Context remaining = resourcesAfterDangerousEntry(resources, entry, rocket);
        return knownReturn
                ? DangerousEntryGate.permitKnownReturn(world, initial, entry.state(), movement, remaining)
                : DangerousEntryGate.permit(world, initial, entry.state(), movement, remaining);
    }

    static HazardRecovery.Context resourcesAfterDangerousEntry(HazardRecovery.Context resources,
            DangerousEntry entry, boolean rocket) {
        HazardRecovery.Context remaining = resources.afterTicks(entry.elapsedTicks(), rocket);
        if (!rocket) return remaining;
        // A request consumes inventory and starts cooldown, but delayed acknowledgement
        // cannot promise thrust at the entry checkpoint. Recovery must work without it.
        return new HazardRecovery.Context(remaining.rocketFlightDurations(), remaining.cooldownTicks(),
                false, remaining.memory(), 0);
    }

    /** A terminal maneuver is revalidated against the exact selected physical curve,
     * not the navigation centreline that the safe braking curve intentionally leaves. */
    private static WaypointFlightTarget executionCorridor(World world, MovementParameters movement,
            List<FlightControlAdmission.Step> prefix, List<FlightControlAdmission.Step> reserve,
            List<FlightControlAdmission.Step> bridge, boolean powered, Vector3d goal,
            boolean landing) {
        var paths = new ArrayList<List<Vector3d>>();
        paths.add(executionPath(world, movement, prefix, powered));
        if (!reserve.isEmpty()) paths.add(executionPath(world, movement, reserve, powered));
        if (!bridge.isEmpty()) paths.add(executionPath(world, movement, bridge, powered));
        if (landing && !powered) {
            var control = prefix.get(0).control();
            var state = replay(world, movement, prefix.get(0).expected(), control, false,
                    prefix.size(), goal, true).end();
            var continuation = replay(world, movement, state, control, false, 80, goal, true);
            if (normalLanding(world, continuation.end(), goal))
                paths.add(executionPath(world, movement, continuation.steps(), false));
            var recovery = new ElytraFlightController.Decision(control.yaw(), -55, false,
                    ElytraFlightController.Phase.CRUISE);
            var recovered = replay(world, movement, state, recovery, false,
                    RESERVE_TICKS, goal, false);
            if (!recovered.steps().isEmpty())
                paths.add(executionPath(world, movement, recovered.steps(), false));
        }
        return WaypointFlightTarget.executionWitness(paths, goal, landing);
    }

    private static List<Vector3d> executionPath(World world, MovementParameters movement,
            List<FlightControlAdmission.Step> steps, boolean powered) {
        var points = new ArrayList<Vector3d>(steps.size() + 1);
        points.add(steps.get(0).expected().position());
        for (var step : steps) {
            var result = new GlidingPhysics(step.control().yaw(), step.control().pitch(), powered,
                    PlayerInput.NONE, movement).step(world, step.expected(),
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            points.add(result.position());
        }
        return List.copyOf(points);
    }

    /** Fixed, bounded arrival candidates; only exact terminal replay may admit ground contact. */
    static List<FlightControlAdmission.Step> arrival(World world, MovementParameters movement,
            PhysicsState state, Vector3d goal, boolean attached, float fallbackHeading) {
        if (attached) return attachedArrivalApproach(world, movement, state, goal, fallbackHeading);
        float forward = heading(state.position(), goal, fallbackHeading);
        List<FlightControlAdmission.Step> best = List.of();
        double bestScore = Double.NEGATIVE_INFINITY;
        for (float turn : new float[]{0, -30, 30, -90, 90, 180}) {
            for (float pitch : new float[]{-60, -30, 0, 20, 40, 60, 85}) {
                var control = new ElytraFlightController.Decision(forward + turn, pitch, false,
                        ElytraFlightController.Phase.LANDING);
                var candidate = replay(world, movement, state, control, false, 40, goal, true);
                if (candidate.steps().isEmpty()) continue;
                boolean landed = normalLanding(world, candidate.end(), goal);
                // A constant turn away from the goal is useful only when this exact replay
                // closes on the goal. Publishing its unfinished prefix under receding-horizon
                // replanning repeatedly restarts the brake and can fly back off safe support.
                if (!landed && Math.cos(Math.toRadians(turn)) <= 0) continue;
                // A truncated collision approach is not an arrival plan.
                if (!landed && candidate.steps().size() < 40) continue;
                Vector3d end = candidate.end().position();
                double score = (landed ? 1000 : 0) - end.distance(goal)
                        - candidate.end().velocity().length() * 2;
                if (score > bestScore) {
                    bestScore = score;
                    best = List.copyOf(candidate.steps().subList(0, Math.min(HORIZON, candidate.steps().size())));
                }
            }
        }
        return best;
    }

    /** Existing thrust may brake toward a supported goal but cannot authorize touchdown. */
    private static List<FlightControlAdmission.Step> attachedArrivalApproach(World world,
            MovementParameters movement, PhysicsState state, Vector3d goal, float fallbackHeading) {
        float forward = heading(state.position(), goal, fallbackHeading);
        List<FlightControlAdmission.Step> best = List.of();
        double bestScore = Double.NEGATIVE_INFINITY;
        for (float turn : new float[]{0, -30, 30, -90, 90, 180}) {
            // Reversing attached thrust is not braking: it erases forward glide
            // range before the server-reported attachment can expire.
            if (Math.cos(Math.toRadians(turn)) <= 0) continue;
            for (float pitch : new float[]{-60, -30, -10, 0, 20, 40, 60, 85}) {
                var control = new ElytraFlightController.Decision(forward + turn, pitch, false,
                        ElytraFlightController.Phase.CRUISE);
                var candidate = replay(world, movement, state, control, true, 40, goal, false);
                if (candidate.steps().size() < 40
                        || candidate.end().fallDistance() >= movement.safeFallDistance()
                        || candidate.steps().stream().anyMatch(step -> step.expected().onGround()
                                || step.expected().fallDistance() >= movement.safeFallDistance())) continue;
                double score = -candidate.end().position().distance(goal)
                        - candidate.end().velocity().length() * 2;
                if (score > bestScore) {
                    bestScore = score;
                    best = List.copyOf(candidate.steps().subList(0, HORIZON));
                }
            }
        }
        return best;
    }

    /** Eight blocks of vertical maneuvering room, or .8 blocks/tick of airspeed,
     * are cruise targets, not permission to cross unobserved terrain. */
    public static boolean needsLavaMargin(World world, PhysicsState state) {
        if (state.onGround()) return false;
        var p = state.position();
        boolean low = world.readTerrain(() -> {
            int x = (int) Math.floor(p.x), z = (int) Math.floor(p.z);
            if (!world.chunkLoaded(x >> 4, z >> 4)) return false;
            for (int y = (int) Math.floor(p.y); y >= Math.floor(p.y - 8); y--) {
                var cell = new Vector3d(x + .5, y, z + .5);
                String name = world.getBlockStateAt(cell).blockName();
                if (name.equals("lava") || name.equals("minecraft:lava")) return true;
                if (world.isSolid(cell)) return false;
            }
            return false;
        });
        // Nearby lava must not turn a slow, supported shore approach into another
        // boost. Check the whole footprint, retaining rescue at an exposed edge.
        return low || Math.hypot(state.velocity().x, state.velocity().z) < .8
                && KnownLandingReserve.nearLava(world, state)
                && DangerousEntryGate.exposed(world, state);
    }

    /** Finite terrain-checked rescue; never blindly fire into a roof or unknown chunk. */
    static List<FlightControlAdmission.Step> poweredEscape(World world, MovementParameters movement,
            PhysicsState state, float forward, Vector3d goal) {
        return poweredEscape(world, movement, state, forward, goal, false);
    }

    private static List<FlightControlAdmission.Step> poweredEscape(World world, MovementParameters movement,
            PhysicsState state, float forward, Vector3d goal, boolean preserveFallMargin) {
        List<FlightControlAdmission.Step> best = List.of();
        double bestScore = Double.NEGATIVE_INFINITY;
        for (float turn : new float[]{0, -30, 30, -60, 60, -90, 90, 180}) {
            for (float pitch : new float[]{-35, -20, -10, 0}) {
                var control = new ElytraFlightController.Decision(forward + turn, pitch, true,
                        ElytraFlightController.Phase.CRUISE);
                var powered = replay(world, movement, state, control, true, HORIZON, goal, false,
                        null, preserveFallMargin);
                if (powered.steps().size() < HORIZON) continue;
                var stalled = replay(world, movement, state, control, false, HORIZON, goal, false,
                        null, preserveFallMargin);
                int count = Math.min(powered.steps().size(), stalled.steps().size());
                if (count == 0) continue;
                double score = (powered.end().position().y - state.position().y) * 2
                        + Math.hypot(powered.end().velocity().x, powered.end().velocity().z)
                        - Math.abs(turn) * .01;
                if (score > bestScore) {
                    bestScore = score;
                    best = withFirstRocket(powered.steps().subList(0, count), true);
                }
            }
        }
        return best;
    }

    static boolean preferPowered(int dualBranchTicks, double poweredScore,
            LocalLookaheadSolver.PrefixResult unpowered) {
        return dualBranchTicks > 0 && (dualBranchTicks > unpowered.prefix().size()
                || dualBranchTicks == unpowered.prefix().size() && poweredScore > unpowered.score());
    }

    private static List<FlightControlAdmission.Step> withFirstRocket(List<FlightControlAdmission.Step> steps,
            boolean rocket) {
        if (!rocket || steps.isEmpty()) return steps;
        var result = new ArrayList<>(steps);
        var first = result.get(0);
        var control = first.control();
        result.set(0, new FlightControlAdmission.Step(new ElytraFlightController.Decision(control.yaw(),
                control.pitch(), true, control.phase()), first.expected()));
        return List.copyOf(result);
    }

    record Replay(List<FlightControlAdmission.Step> steps, PhysicsState end) {}
    static Replay replay(World world, MovementParameters movement, PhysicsState initial,
            ElytraFlightController.Decision control, boolean attached, int ticks, Vector3d goal,
            boolean landing) {
        return replay(world, movement, initial, control, attached, ticks, goal, landing, null);
    }

    private static Replay replay(World world, MovementParameters movement, PhysicsState initial,
            ElytraFlightController.Decision control, boolean attached, int ticks, Vector3d goal,
            boolean landing, WaypointFlightTarget corridor) {
        return replay(world, movement, initial, control, attached, ticks, goal, landing, corridor, false);
    }

    private static Replay replay(World world, MovementParameters movement, PhysicsState initial,
            ElytraFlightController.Decision control, boolean attached, int ticks, Vector3d goal,
            boolean landing, WaypointFlightTarget corridor, boolean preserveFallMargin) {
        return replay(world, movement, initial, control, attached, ticks, goal, landing, corridor,
                preserveFallMargin, NO_CANCELLATION);
    }

    private static Replay replay(World world, MovementParameters movement, PhysicsState initial,
            ElytraFlightController.Decision control, boolean attached, int ticks, Vector3d goal,
            boolean landing, WaypointFlightTarget corridor, boolean preserveFallMargin, Runnable validateRequest) {
        PhysicsState state = initial;
        var steps = new ArrayList<FlightControlAdmission.Step>();
        var noRocket = new ElytraFlightController.Decision(control.yaw(), control.pitch(), false, control.phase());
        var physics = new GlidingPhysics(control.yaw(), control.pitch(), attached, PlayerInput.NONE, movement);
        var travelPhysics = new GlidingPhysics(control.yaw(), control.pitch(), false, PlayerInput.NONE, movement);
        for (int i = 0; i < ticks; i++) {
            validateRequest.run();
            var next = physics.step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            // Attached thrust is applied after travel and masks the zero Y velocity
            // produced by ceiling clipping. Inspect travel from this SAME powered
            // state, not an independently evolving unpowered trajectory.
            var travel = attached ? travelPhysics.step(world, state, PlayerPose.HALF_WIDTH,
                    PlayerPose.GLIDING.height()) : next;
            var end = new PhysicsState(next.position(), next.velocity(), next.onGround(), next.fallDistance(),
                    PlayerPose.GLIDING);
            // Reject hazardous arrival descent before touchdown: a long airborne dive
            // can otherwise win arrival scoring and exhaust its safe fall margin.
            // Known-return travel also reserves this strict airborne margin. Ordinary
            // cruise/Follow keep their policy: airborne fall distance alone is not damage.
            if ((landing || preserveFallMargin) && !(next.fallDistance() < movement.safeFallDistance())
                    || next.collidedX() || next.collidedZ()
                    || next.onGround() && attached
                    || state.velocity().y > .003 && travel.velocity().y == 0
                    || !LocalAirRoute.visible(world, state.position(), next.position(), PlayerPose.GLIDING)
                    || corridor != null && !corridor.contains(state.position(), next.position())
                    || !hazardFree(world, state, next)
                    || next.onGround() && (!landing || !normalLanding(world, end, goal)
                        || state.fallDistance() + Math.max(0, state.position().y - next.position().y)
                            > movement.safeFallDistance())) break;
            steps.add(new FlightControlAdmission.Step(noRocket, state));
            state = end;
            if (next.onGround()) break;
        }
        return new Replay(List.copyOf(steps), state);
    }

    private static boolean hazardFree(World world, PhysicsState before, PhysicsResult after) {
        var gliding = new PhysicsResult(after.position(), after.velocity(), after.collidedX(),
                after.collidedZ(), after.onGround(), after.fallDistance(), PlayerPose.GLIDING);
        return FlightHazards.safeTravel(world, before, gliding);
    }

    /** Being on ground alone is never goto success (wrong height, unsafe support and void fail). */
    public static boolean normalLanding(World world, PhysicsState state, Vector3d goal) {
        return WaypointFlightTarget.normalLanding(world, state, goal);
    }

    private static float heading(Vector3d from, Vector3d to, float fallback) {
        double x = to.x - from.x, z = to.z - from.z;
        return x == 0 && z == 0 ? fallback : (float) Math.toDegrees(Math.atan2(-x, z));
    }

    /** Record the movement thread's desired geometric checkpoint even while a worker is busy.
     * Already admitted controls retain their original corridor; only a worker-proven episode may
     * transfer physical authority to this route.
     */
    public void observeRoute(MovementSync sync, Object owner, WaypointFlightTarget route) {
        Objects.requireNonNull(sync);
        Objects.requireNonNull(owner);
        Objects.requireNonNull(route);
        synchronized (sync) {
            observeRoute(route.corridorSnapshot());
        }
    }

    /** Immediately revoke publication rights when local route ownership is dropped. */
    public void invalidateRoute(MovementSync sync, Object owner) {
        Objects.requireNonNull(sync);
        Objects.requireNonNull(owner);
        synchronized (sync) {
            requestedRoute = null;
            routeRevision++;
            sync.invalidateForwardFlightEmergency(owner);
        }
    }

    /** Caller holds the MovementSync monitor. */
    private long observeRoute(WaypointFlightTarget route) {
        if (requestedRoute == null || !requestedRoute.sameCorridor(route)) {
            requestedRoute = route;
            routeRevision++;
        }
        return routeRevision;
    }

    /** Check ownership briefly; terrain and physics always run outside this monitor. */
    private void validateRequest(MovementSync sync, FlightControlAdmission.Source source,
            World world, long requestRouteRevision) {
        synchronized (sync) {
            if (closed || routeRevision != requestRouteRevision || sync.getWorld() != world
                    || !current(sync, source)) throw new ObsoleteRequestException();
        }
    }

    private static final class ObsoleteRequestException extends RuntimeException {
        private ObsoleteRequestException() { super(null, null, false, false); }
    }

    private static boolean current(MovementSync sync, FlightControlAdmission.Source source) {
        long age = sync.physicsTickVersion() - source.tick();
        return source.generation() == sync.getNavigationGeneration() && source.epoch() == sync.flightControlEpoch()
                && age >= 0 && age <= 8;
    }

    /** Bounded planner-side join; never call while holding the physics monitor. */
    public boolean awaitIdle(long timeout, TimeUnit unit) throws Exception {
        CompletableFuture<Void> pending;
        synchronized (this) { pending = completion; }
        pending.get(timeout, unit);
        return true;
    }

    @Override public synchronized void close() {
        closed = true;
        worker.shutdown();
    }
}
