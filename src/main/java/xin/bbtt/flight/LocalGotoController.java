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
    private static final int HORIZON = 12;
    private static final int RESERVE_TICKS = 6;
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
    private CompletableFuture<Void> completion = CompletableFuture.completedFuture(null);
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
        synchronized (this) {
            if (closed || busy || source.tick() == lastTick && source.epoch() == lastEpoch
                    && source.generation() == lastGeneration
                    && (source.tick() != 0 || source.state().equals(lastState))) return false;
            World world;
            MovementParameters movement;
            float yaw, pitch;
            boolean attached;
            synchronized (sync) {
                if (!current(sync, source)) return false;
                world = sync.getWorld();
                movement = sync.getMovementParameters();
                yaw = sync.yaw.get();
                pitch = sync.pitch.get();
                attached = sync.hasAttachedFirework();
                float forward = heading(source.state().position(), snapshot.position(), yaw);
                if (!sync.registerForwardFlightEmergency(owner, source, forward, snapshot)) return false;
            }
            busy = true;
            lastTick = source.tick();
            lastState = source.state();
            lastEpoch = source.epoch();
            lastGeneration = source.generation();
            var finished = new CompletableFuture<Void>();
            completion = finished;
            worker.execute(() -> {
                try {
                    compute(sync, owner, source, snapshot, endpoint, world, movement, yaw, pitch,
                            attached, allowedRocket, accepted, tracking, resources, allowExposedProgress, knownReturn);
                    synchronized (LocalGotoController.this) { busy = false; }
                    finished.complete(null);
                } catch (Throwable failure) {
                    synchronized (LocalGotoController.this) { busy = false; }
                    finished.completeExceptionally(failure);
                }
            });
        }
        return true;
    }

    private void compute(MovementSync sync, Object owner, FlightControlAdmission.Source source,
            WaypointFlightTarget route, Vector3d goal, World world, MovementParameters movement,
            float yaw, float pitch, boolean attached, boolean allowedRocket, Runnable accepted,
            LocalLookaheadSolver.MovingTarget tracking, HazardRecovery.Context resources,
            boolean allowExposedProgress, boolean knownReturn) {
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
        Vector3d forwardPoint = lookahead.isEmpty() ? points.get(points.size() - 1) : lookahead.get(0);
        float forward = heading(source.state().position(), forwardPoint, yaw);
        List<FlightControlAdmission.Step> prefix = List.of();
        List<FlightControlAdmission.Step> reserve = List.of();
        LocalLookaheadSolver.PrefixResult unpowered = null, powered = null;
        boolean rocket = false;
        boolean terminalArrival = false;
        boolean requireAirborne = tracking != null || route.partial() || !route.landingIntent();
        Status status = Status.UNPROVEN;
        if (tracking == null && normalLanding(world, source.state(), goal)) {
            status = Status.LANDED;
        } else if (!source.state().onGround()) {
            // Even an empty visible lookahead is not permission to fly blindly. The solver still
            // checks every executable step towards the route endpoint and may return UNPROVEN.
            var anchors = lookahead.isEmpty() ? List.of(forwardPoint) : lookahead;
            // Track the live endpoint only when the shared body sweep can see it. Behind
            // terrain, route progress owns steering until the endpoint becomes visible.
            boolean trackingVisible = tracking != null
                    && LocalAirRoute.visible(world, source.state().position(), goal, PlayerPose.GLIDING);
            if (trackingVisible) anchors = List.of(goal);
            // Visibility limits steering anchors, not pursuit urgency. Beyond the bounded
            // visibility sweep, dropping tracking makes short route anchors penalize thrust
            // for overshooting the lookahead and leaves a distant target unreacquired.
            var scoringTarget = tracking;
            if (tracking != null && !trackingVisible && !lookahead.isEmpty()) {
                // A blocked pursuit ray must not reward staying below route clearance.
                // Bound the probe, so a distant unobstructed target retains pursuit urgency
                // even when the full endpoint is beyond the visibility sweep cap.
                Vector3d ray = new Vector3d(goal).sub(source.state().position());
                double distance = ray.length();
                Vector3d near = new Vector3d(ray).mul(distance > 48 ? (48 - 1e-6) / distance : 1)
                        .add(source.state().position());
                if (!LocalAirRoute.visible(world, source.state().position(), near, PlayerPose.GLIDING)) {
                    Vector3d routedAim = tracking.position();
                    routedAim.y = Math.max(routedAim.y, forwardPoint.y);
                    scoringTarget = new LocalLookaheadSolver.MovingTarget(routedAim, tracking.velocity());
                }
            }
            var solver = new LocalLookaheadSolver();
            var budget = new LocalLookaheadSolver.Budget(600, HORIZON, 25_000_000);
            unpowered = solver.solvePrefix(anchors, source.state(), world, movement, yaw, pitch, attached, budget, scoringTarget);
            prefix = unpowered.prefix();
            boolean supportedGoal = tracking == null && normalLanding(world,
                    new PhysicsState(goal, new Vector3d(), true, 0, PlayerPose.STANDING), goal);
            // Height overshoot is descent, not another cruise leg. A spherical arrival
            // radius lets a supported goal directly below us trigger repeated boosts.
            boolean arriving = tracking == null && (source.state().position().distance(goal) < 32
                    || supportedGoal && source.state().position().y >= goal.y
                    && Math.hypot(source.state().position().x - goal.x,
                            source.state().position().z - goal.z) < 32);
            boolean needsClimb = source.state().position().y < goal.y - 1;
            terminalArrival = supportedGoal && arriving && !needsClimb;
            if (allowedRocket && !attached && accepted != null && (!arriving || needsClimb)) {
                powered = solver.solvePrefix(anchors, source.state(), world, movement, yaw, pitch, true, budget, scoringTarget);
                if (!powered.prefix().isEmpty()) {
                    // Different winning candidates are not a dual-branch proof: replay the SAME
                    // powered winner without thrust before authorizing the first-step action.
                    var control = powered.decision();
                    var stalled = replay(world, movement, source.state(), control, false,
                            powered.prefix().size(), goal, false);
                    int count = Math.min(stalled.steps().size(), powered.prefix().size());
                    // The stalled branch bounds the executable commitment, not the powered
                    // benefit. Comparing its truncated horizon to unpowered rejects a launch
                    // rescue whenever both no-thrust branches reach ground on the same tick.
                    // Keep both branches safe for count ticks; rank the powered continuation
                    // by its own horizon before progress, as the prefix solver does.
                    if (count > 0 && preferPowered(powered.prefix().size(), powered.score(), unpowered)) {
                        var chosen = new ArrayList<>(powered.prefix().subList(0, count));
                        var first = chosen.get(0);
                        chosen.set(0, new FlightControlAdmission.Step(new ElytraFlightController.Decision(
                                control.yaw(), control.pitch(), true, control.phase()), first.expected()));
                        prefix = List.copyOf(chosen);
                        rocket = true;
                    }
                }
            }
            // Preserve maneuvering margin over lava rather than matching a dangerously
            // low route at minimum speed. Follow/End never opt in to this policy.
            boolean lavaMargin = knownReturn && tracking == null && needsLavaMargin(world, source.state());
            if (lavaMargin) terminalArrival = false;
            if (lavaMargin && (attached || allowedRocket && accepted != null)) {
                // Existing thrust still needs escape steering: the ordinary solver may
                // track a descending route straight back toward lava. Do not stack a
                // rocket; retain the escape's same-control no-thrust commitment bound.
                var escape = poweredEscape(world, movement, source.state(), forward, goal);
                if (!escape.isEmpty()) {
                    prefix = escape;
                    rocket = !attached;
                }
            }
            if (arriving && !needsClimb && !lavaMargin
                    && normalLanding(world, new PhysicsState(goal, new Vector3d(), true, 0,
                            PlayerPose.STANDING), goal)) {
                var arrival = arrival(world, movement, source.state(), goal, attached, forward);
                if (!arrival.isEmpty()) {
                    prefix = arrival;
                    rocket = false;
                }
            }
            // Keep a terminal prefix within the admission revalidation horizon. Every later
            // cursor then reaches the same checkpoint before its exact landing/recovery proof.
            if (terminalArrival && !rocket && !prefix.isEmpty()
                    && prefix.get(0).control().phase() == ElytraFlightController.Phase.LANDING
                    && prefix.size() > RESERVE_TICKS)
                prefix = List.copyOf(prefix.subList(0, RESERVE_TICKS));
            // A finite non-rocket braking preview, not a terminal recovery certificate.
            if (!prefix.isEmpty()) {
                var selected = prefix.get(0).control();
                var trajectory = replay(world, movement, source.state(), selected, attached || rocket,
                        prefix.size(), goal, arriving && !rocket);
                prefix = withFirstRocket(trajectory.steps(), rocket);
                if (!prefix.isEmpty()) {
                    var brake = new ElytraFlightController.Decision(selected.yaw(), -45, false,
                            ElytraFlightController.Phase.CRUISE);
                    reserve = replay(world, movement, trajectory.end(), brake, attached || rocket,
                            RESERVE_TICKS, goal, tracking == null).steps();
                    status = Status.SAFE_PREFIX;
                }
            }
            if (prefix.isEmpty()) {
                // The generic solver deliberately excludes ground contact; bounded goal-only
                // braking may include a harmless supported touchdown, never arbitrary ground.
                var brake = new ElytraFlightController.Decision(forward, -45, false,
                        tracking == null ? ElytraFlightController.Phase.LANDING
                                : ElytraFlightController.Phase.CRUISE);
                prefix = replay(world, movement, source.state(), brake, attached,
                        RESERVE_TICKS, goal, tracking == null).steps();
                rocket = false;
                if (!prefix.isEmpty()) status = Status.SAFE_PREFIX;
            }
        }
        if (!prefix.isEmpty()) {
            var chosen = prefix.get(0).control();
            var end = replay(world, movement, source.state(), chosen, attached || rocket,
                    prefix.size(), goal, chosen.phase() == ElytraFlightController.Phase.LANDING).end();
            var entry = end;
            if (!DangerousEntryGate.exposed(world, source.state())) {
                for (var step : prefix) {
                    if (DangerousEntryGate.exposed(world, step.expected())) {
                        entry = step.expected();
                        break;
                    }
                }
            }
            if ((!allowExposedProgress && !DangerousEntryGate.exposed(world, source.state())
                    && DangerousEntryGate.exposed(world, entry))
                    || !(knownReturn
                            ? DangerousEntryGate.permitKnownReturn(world, source.state(), entry, movement,
                                    resources.afterTicks(prefix.size(), rocket))
                            : DangerousEntryGate.permit(world, source.state(), entry, movement,
                                    resources.afterTicks(prefix.size(), rocket)))) {
                prefix = List.of();
                reserve = List.of();
                rocket = false;
                status = Status.UNPROVEN;
            }
        }
        synchronized (sync) {
            if (closed || sync.getWorld() != world || !current(sync, source)
                    || !sync.refineForwardFlightEmergency(owner, source, forward, route)) return;
            if (!prefix.isEmpty()) {
                var selected = prefix.get(0).control();
                // Use the admitted heading for the use-item packet, including return turns.
                // A dynamic endpoint is not permission to remove geometric admission. Its
                // already terrain-checked fixed prefix gets an exact immutable witness instead
                // of inheriting an obsolete endpoint corridor. Static routes retain their
                // planner corridor; goal-arrival maneuvers use their exact execution witness.
                WaypointFlightTarget admissionRoute = route;
                if (tracking != null || terminalArrival && !rocket)
                    admissionRoute = executionCorridor(world, movement, prefix, reserve,
                            attached || rocket, goal, terminalArrival && !rocket);
                sync.publishFlightPrefix(owner, source, prefix, reserve,
                        admissionRoute, rocket ? () -> {
                    sync.setYawToward(selected.yaw());
                    sync.pitch.set(selected.pitch());
                    accepted.run();
                } : () -> {}, requireAirborne);
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

    /** A terminal maneuver is revalidated against the exact selected physical curve,
     * not the navigation centreline that the safe braking curve intentionally leaves. */
    private static WaypointFlightTarget executionCorridor(World world, MovementParameters movement,
            List<FlightControlAdmission.Step> prefix, List<FlightControlAdmission.Step> reserve,
            boolean powered, Vector3d goal, boolean landing) {
        var paths = new ArrayList<List<Vector3d>>();
        paths.add(executionPath(world, movement, prefix, powered));
        if (!reserve.isEmpty()) paths.add(executionPath(world, movement, reserve, powered));
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

    /** Fixed, bounded arrival candidates; only exact replay may admit ground contact. */
    static List<FlightControlAdmission.Step> arrival(World world, MovementParameters movement,
            PhysicsState state, Vector3d goal, boolean attached, float fallbackHeading) {
        float forward = heading(state.position(), goal, fallbackHeading);
        List<FlightControlAdmission.Step> best = List.of();
        double bestScore = Double.NEGATIVE_INFINITY;
        for (float turn : new float[]{0, -30, 30, -90, 90, 180}) {
            // Reversing attached thrust is not braking: it erases forward glide
            // range before the server-reported attachment can expire.
            if (attached && Math.cos(Math.toRadians(turn)) <= 0) continue;
            for (float pitch : new float[]{-60, -30, 0, 20, 40, 60, 85}) {
                var control = new ElytraFlightController.Decision(forward + turn, pitch, false,
                        ElytraFlightController.Phase.LANDING);
                var candidate = replay(world, movement, state, control, attached, 40, goal, true);
                if (candidate.steps().isEmpty()) continue;
                boolean landed = normalLanding(world, candidate.end(), goal);
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
        List<FlightControlAdmission.Step> best = List.of();
        double bestScore = Double.NEGATIVE_INFINITY;
        for (float turn : new float[]{0, -30, 30, -60, 60, -90, 90, 180}) {
            for (float pitch : new float[]{-35, -20, -10, 0}) {
                var control = new ElytraFlightController.Decision(forward + turn, pitch, true,
                        ElytraFlightController.Phase.CRUISE);
                var powered = replay(world, movement, state, control, true, HORIZON, goal, false);
                if (powered.steps().size() < HORIZON) continue;
                var stalled = replay(world, movement, state, control, false, HORIZON, goal, false);
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
        PhysicsState state = initial;
        var steps = new ArrayList<FlightControlAdmission.Step>();
        var noRocket = new ElytraFlightController.Decision(control.yaw(), control.pitch(), false, control.phase());
        var physics = new GlidingPhysics(control.yaw(), control.pitch(), attached, PlayerInput.NONE, movement);
        var travelPhysics = new GlidingPhysics(control.yaw(), control.pitch(), false, PlayerInput.NONE, movement);
        for (int i = 0; i < ticks; i++) {
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
            // Cruise/Follow retain their airborne replay policy; fall distance alone
            // is not touchdown damage while gliding toward a moving target.
            if (landing && next.fallDistance() >= movement.safeFallDistance()
                    || next.collidedX() || next.collidedZ()
                    || next.onGround() && attached
                    || state.velocity().y > .003 && travel.velocity().y == 0
                    || !LocalAirRoute.visible(world, state.position(), next.position(), PlayerPose.GLIDING)
                    || !hazardFree(world, state.position(), next.position())
                    || next.onGround() && (!landing || !normalLanding(world, end, goal)
                        || state.fallDistance() + Math.max(0, state.position().y - next.position().y)
                            > movement.safeFallDistance())) break;
            steps.add(new FlightControlAdmission.Step(noRocket, state));
            state = end;
            if (next.onGround()) break;
        }
        return new Replay(List.copyOf(steps), state);
    }

    /** Conservative enclosing-box hazard scan, bounded independently of malformed speed. */
    private static boolean hazardFree(World world, Vector3d from, Vector3d to) {
        double r = PlayerPose.HALF_WIDTH;
        int minX = (int) Math.floor(Math.min(from.x, to.x) - r - 1e-7);
        int maxX = (int) Math.floor(Math.max(from.x, to.x) + r);
        int minY = (int) Math.floor(Math.min(from.y, to.y) - 1e-7);
        int maxY = (int) Math.floor(Math.max(from.y, to.y) + PlayerPose.GLIDING.height());
        int minZ = (int) Math.floor(Math.min(from.z, to.z) - r - 1e-7);
        int maxZ = (int) Math.floor(Math.max(from.z, to.z) + r);
        if ((long) (maxX - minX + 1) * (maxY - minY + 1) * (maxZ - minZ + 1) > 4096) return false;
        for (int x = minX; x <= maxX; x++) for (int y = minY; y <= maxY; y++)
            for (int z = minZ; z <= maxZ; z++)
                if (!FlightHazards.safeLanding(world, new Vector3d(x + .5, y, z + .5))) return false;
        return true;
    }

    /** Being on ground alone is never goto success (wrong height, unsafe support and void fail). */
    public static boolean normalLanding(World world, PhysicsState state, Vector3d goal) {
        return WaypointFlightTarget.normalLanding(world, state, goal);
    }

    private static float heading(Vector3d from, Vector3d to, float fallback) {
        double x = to.x - from.x, z = to.z - from.z;
        return x == 0 && z == 0 ? fallback : (float) Math.toDegrees(Math.atan2(-x, z));
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
