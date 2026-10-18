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
    private static final double LANDING_RADIUS = 2;
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
        Objects.requireNonNull(sync);
        Objects.requireNonNull(owner);
        Objects.requireNonNull(source);
        Objects.requireNonNull(route);
        Vector3d endpoint = new Vector3d(goal);
        if (!endpoint.isFinite()) throw new IllegalArgumentException("Non-finite goal");
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
        }
        var snapshot = new WaypointFlightTarget(route.waypoints(), route.goal(), route.landed(),
                route.partial(), route.corridorRadius());
        synchronized (this) {
            if (closed || busy || source.tick() == lastTick && source.epoch() == lastEpoch
                    && source.generation() == lastGeneration
                    && (source.tick() != 0 || source.state().equals(lastState))) return false;
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
                            attached, allowedRocket, accepted, tracking);
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
            LocalLookaheadSolver.MovingTarget tracking) {
        var points = route.waypoints();
        if (workerRoute == null || workerEpoch != source.epoch() || workerGeneration != source.generation()
                || !points.equals(workerPoints)) {
            workerRoute = new LocalAirRoute(route);
            workerPoints = points;
            workerEpoch = source.epoch();
            workerGeneration = source.generation();
        }
        var lookahead = workerRoute.lookahead(world, source.state());
        Vector3d forwardPoint = lookahead.isEmpty() ? points.get(points.size() - 1) : lookahead.get(0);
        float forward = heading(source.state().position(), forwardPoint, yaw);
        List<FlightControlAdmission.Step> prefix = List.of();
        List<FlightControlAdmission.Step> reserve = List.of();
        LocalLookaheadSolver.PrefixResult unpowered = null, powered = null;
        boolean rocket = false;
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
            boolean arriving = tracking == null && source.state().position().distance(goal) < 32;
            boolean needsClimb = source.state().position().y < goal.y - 1;
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
            if (arriving && !needsClimb) {
                var arrival = arrival(world, movement, source.state(), goal, attached, forward);
                if (!arrival.isEmpty()) {
                    prefix = arrival;
                    rocket = false;
                }
            }
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
            var resources = new HazardRecovery.Context(new int[0], 0, false, null);
            var entry = end;
            if (!DangerousEntryGate.exposed(world, source.state())) {
                for (var step : prefix) {
                    if (DangerousEntryGate.exposed(world, step.expected())) {
                        entry = step.expected();
                        break;
                    }
                }
            }
            if (!DangerousEntryGate.permit(world, source.state(), entry, movement, resources)) {
                prefix = List.of();
                reserve = List.of();
                rocket = false;
                status = Status.UNPROVEN;
            }
        }
        synchronized (sync) {
            if (closed || sync.getWorld() != world || !current(sync, source)) return;
            sync.registerForwardFlightEmergency(owner, source, forward);
            if (!prefix.isEmpty()) sync.publishFlightPrefix(owner, source, prefix, reserve, null,
                    rocket ? accepted : () -> {});
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

    /** Fixed, bounded arrival candidates; only exact replay may admit ground contact. */
    private static List<FlightControlAdmission.Step> arrival(World world, MovementParameters movement,
            PhysicsState state, Vector3d goal, boolean attached, float forward) {
        List<FlightControlAdmission.Step> best = List.of();
        double bestScore = Double.NEGATIVE_INFINITY;
        for (float turn : new float[]{0, -30, 30, -90, 90, 180}) {
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
        for (int i = 0; i < ticks; i++) {
            var next = physics.step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            var end = new PhysicsState(next.position(), next.velocity(), next.onGround(), next.fallDistance(),
                    PlayerPose.GLIDING);
            if (next.collidedX() || next.collidedZ() || state.velocity().y > .003 && next.velocity().y == 0
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
        Vector3d p = state.position();
        double r = PlayerPose.HALF_WIDTH;
        return state.onGround() && p.isFinite() && goal.isFinite()
                && Math.abs(p.y - goal.y) <= 1 && Math.hypot(p.x - goal.x, p.z - goal.z) <= LANDING_RADIUS
                && LocalAirRoute.visible(world, p, p, PlayerPose.STANDING)
                && FlightHazards.safeLanding(world, p)
                && world.isBoxColliding(p.x - r, p.y - .05, p.z - r, p.x + r, p.y, p.z + r);
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
