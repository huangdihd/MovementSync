package xin.bbtt.physics;

import java.util.List;
import java.util.Objects;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Geometric route and movement-thread-owned cursor; never a replay of flight inputs. */
public final class WaypointFlightTarget implements FlightTarget {
    private final List<Vector3d> waypoints;
    private final Vector3d goal;
    private final boolean landing;
    private final boolean partial;
    private final double corridorRadius;
    private final World geometry;
    private final List<List<Vector3d>> executionPaths;
    private int index;
    private boolean pursuit;
    private FlightTarget followed;
    private Vector3d sampledAim;

    WaypointFlightTarget asPursuit() { pursuit = true; return this; }
    public boolean pursuit() { return pursuit; }
    public boolean pursuitEndpoint() { return pursuit && !partial && index == waypoints.size() - 1; }

    /** Bind on the movement thread; workers never sample mutable entity state. */
    public void follow(FlightTarget target, Vector3d endpoint) {
        followed = target;
        sampledAim = new Vector3d(endpoint);
    }

    public static Vector3d pursuitAim(FlightTarget target, PhysicsState observer) {
        Vector3d aim = target.aim(observer);
        Vector3d velocity = target.velocity();
        double speed = Math.hypot(velocity.x, velocity.z);
        if (speed > 0) {
            double distance = Math.hypot(target.position().x - observer.position().x,
                    target.position().z - observer.position().z);
            double lead = distance / (Math.hypot(observer.velocity().x, observer.velocity().z) + speed);
            aim.fma(-lead, velocity);
        }
        return aim;
    }

    @Override public Vector3d aim(PhysicsState observer) {
        if (!pursuit || followed == null || partial || index != waypoints.size() - 1) return position();
        Vector3d live = position().add(pursuitAim(followed, observer).sub(sampledAim));
        // A moving endpoint is not permission to leave the validated corridor or
        // cut through a newly encountered ridge while the replacement is pending.
        return geometry == null || contains(observer.position(), live)
                && LowAltitudePlanner.lineOfSight(geometry, observer.position(), live, 0, 0)
                ? live : position();
    }

    public WaypointFlightTarget(List<Vector3d> waypoints, Vector3d goal, boolean landing) {
        this(waypoints, goal, landing, false, LowAltitudePlanner.HALF_WIDTH);
    }

    public WaypointFlightTarget(List<Vector3d> waypoints, Vector3d goal, boolean landing,
            boolean partial, double corridorRadius) {
        this(waypoints, goal, landing, partial, corridorRadius, null);
    }

    WaypointFlightTarget(List<Vector3d> waypoints, Vector3d goal, boolean landing,
            boolean partial, double corridorRadius, World geometry) {
        this(waypoints, goal, landing, partial, corridorRadius, geometry, List.of());
    }

    private WaypointFlightTarget(List<Vector3d> waypoints, Vector3d goal, boolean landing,
            boolean partial, double corridorRadius, World geometry,
            List<List<Vector3d>> executionPaths) {
        if (waypoints.isEmpty() || !goal.isFinite() || !Double.isFinite(corridorRadius)
                || corridorRadius < 0 || waypoints.stream().anyMatch(p -> !p.isFinite())
                || executionPaths.stream().anyMatch(path -> path.size() < 2
                        || path.stream().anyMatch(p -> !p.isFinite())))
            throw new IllegalArgumentException("Invalid geometric route");
        this.waypoints = waypoints.stream().map(Vector3d::new).toList();
        this.goal = new Vector3d(goal);
        this.landing = landing;
        this.partial = partial;
        this.corridorRadius = corridorRadius;
        this.geometry = geometry;
        this.executionPaths = executionPaths.stream()
                .map(path -> path.stream().map(Vector3d::new).toList()).toList();
        // Planner routes include their starting point as the first corridor vertex.
        this.index = waypoints.size() > 1 ? 1 : 0;
    }

    /** Immutable exact fixed-control trajectories. The epsilon only absorbs replay
     * arithmetic; it is not a navigation or arrival tolerance. */
    public static WaypointFlightTarget executionWitness(List<List<Vector3d>> paths,
            Vector3d goal, boolean landing) {
        if (paths.isEmpty()) throw new IllegalArgumentException("Missing execution path");
        return new WaypointFlightTarget(paths.get(0), goal, landing, false, 0, null, paths);
    }

    public Vector3d goal() { return new Vector3d(goal); }
    public List<Vector3d> waypoints() { return waypoints.stream().map(Vector3d::new).toList(); }
    public int index() { return index; }
    public boolean partial() { return partial; }
    public double corridorRadius() { return corridorRadius; }
    public boolean landingIntent() { return landing && !partial; }
    public boolean executionWitness() { return !executionPaths.isEmpty(); }
    /** Independent cursor snapshot retaining live planner geometry for admission checks. */
    public WaypointFlightTarget corridorSnapshot() {
        var snapshot = new WaypointFlightTarget(waypoints, goal, landing, partial, corridorRadius,
                geometry, executionPaths);
        snapshot.index = index;
        return snapshot;
    }

    /** Geometric checkpoint equality for an asynchronous controller publication. */
    public boolean sameCorridor(WaypointFlightTarget other) {
        return other != null && index == other.index && landing == other.landing
                && partial == other.partial
                && Double.compare(corridorRadius, other.corridorRadius) == 0
                && goal.equals(other.goal) && waypoints.equals(other.waypoints)
                && executionPaths.equals(other.executionPaths);
    }
    /** Planner-backed routes carry geometric terrain validation only, not an executable
     * control or recovery certificate. Budget exhaustion cannot turn this into proof. */
    public boolean terrainValidated() { return geometry != null; }
    @Override public Vector3d position() { return new Vector3d(waypoints.get(index)); }
    @Override public Vector3d velocity() { return pursuit && followed != null ? followed.velocity() : new Vector3d(); }
    @Override public boolean landed() { return !pursuit && landing && !partial && index == waypoints.size() - 1; }
    @Override public boolean lost() { return false; }

    /** Exact supported goto completion; admission and the controller share these bounds. */
    public static boolean normalLanding(World world, PhysicsState state, Vector3d goal) {
        Vector3d p = state.position();
        double r = PlayerPose.HALF_WIDTH;
        return state.onGround() && p.isFinite() && goal.isFinite()
                && Math.abs(p.y - goal.y) <= 1 && Math.hypot(p.x - goal.x, p.z - goal.z) <= 2
                && LocalAirRoute.visible(world, p, p, PlayerPose.STANDING)
                && FlightHazards.safeLanding(world, p)
                && world.isBoxColliding(p.x - r, p.y - .05, p.z - r, p.x + r, p.y, p.z + r);
    }

    public boolean acceptsLanding(World world, PhysicsState state) {
        return landingIntent() && normalLanding(world, state, goal);
    }

    /** A segment must fit in one convex corridor capsule (never cut across a bend).
     * Terrain must additionally be revalidated by the controller's safety prediction. */
    public boolean contains(Vector3d from, Vector3d to) {
        if (!from.isFinite() || !to.isFinite()) return false;
        if (!executionPaths.isEmpty()) return exactContains(from, to);
        if (geometry != null) return expandedContains(from, to);
        if (waypoints.size() == 1) return from.distance(waypoints.get(0)) <= corridorRadius
                && to.distance(waypoints.get(0)) <= corridorRadius;
        for (int i = Math.max(1, index - 1); i < waypoints.size(); i++) {
            // Landing's final connection was validated with body-only clearance.
            double radius = landing && i == waypoints.size() - 1 ? 0 : corridorRadius;
            if (distance(from, waypoints.get(i - 1), waypoints.get(i)) <= radius
                    && distance(to, waypoints.get(i - 1), waypoints.get(i)) <= radius) return true;
        }
        return false;
    }

    private boolean exactContains(Vector3d from, Vector3d to) {
        final double epsilon = 1.0e-5;
        for (var path : executionPaths) for (int i = 1; i < path.size(); i++)
            if (distance(from, path.get(i - 1), path.get(i)) <= epsilon
                    && distance(to, path.get(i - 1), path.get(i)) <= epsilon) return true;
        return false;
    }

    /** The planned space is the bounded, body-visible neighbourhood of the
     * centreline. Validate the whole candidate sweep, including roof/body margins,
     * before admitting an expansion; proximity alone never licenses wall crossing.
     * Queries are live so block changes cannot leave stale clearance certificates. */
    private boolean expandedContains(Vector3d from, Vector3d to) {
        // Each short sweep must belong to one body-visible convex expansion
        // from a single route anchor. Its bounding box proves the entire footprint,
        // not just centre samples, free (conservative at concave corners).
        double length = from.distance(to);
        if (length > 128) return false;
        int steps = Math.max(1, (int) Math.ceil(length * 2));
        Vector3d previous = from;
        for (int step = 1; step <= steps; step++) {
            Vector3d end = new Vector3d(from).lerp(to, (double) step / steps);
            if (!visibleSweep(previous, end)) return false;
            previous = end;
        }
        return true;
    }

    private boolean visibleSweep(Vector3d from, Vector3d to) {
        for (int i = Math.max(1, index - 1); i < waypoints.size(); i++) {
            Vector3d anchor = closest(new Vector3d(from).lerp(to, .5),
                    waypoints.get(i - 1), waypoints.get(i));
            if (from.distance(anchor) > corridorRadius || to.distance(anchor) > corridorRadius) continue;
            Vector3d min = new Vector3d(anchor).min(from).min(to);
            Vector3d max = new Vector3d(anchor).max(from).max(to);
            if (LowAltitudePlanner.clear(geometry, min, max, 0, 0)) return true;
        }
        return false;
    }

    boolean accepts(FlightTrajectory trajectory) {
        Vector3d from = trajectory.initial().position();
        for (int tick = 0; tick < trajectory.size(); tick++) {
            var result = trajectory.result(tick);
            Vector3d to = result.position();
            // Damage-free clipping is not a valid executable route segment.
            if (result.collidedX() || result.collidedZ() || !contains(from, to)) return false;
            from = to;
        }
        return true;
    }

    private static double distance(Vector3d p, Vector3d a, Vector3d b) {
        return p.distance(closest(p, a, b));
    }

    private static Vector3d closest(Vector3d p, Vector3d a, Vector3d b) {
        Vector3d edge = new Vector3d(b).sub(a);
        double t = edge.lengthSquared() == 0 ? 0
                : Math.max(0, Math.min(1, new Vector3d(p).sub(a).dot(edge) / edge.lengthSquared()));
        return edge.mul(t).add(a);
    }

    /** Reconcile a worker's start cursor with the live body, never with elapsed time.
     * Only a body-visible connection to the closest remaining segment may skip
     * vertices; a nearby segment behind a wall is not a publication certificate. */
    public boolean reconcile(World world, PhysicsState observer) {
        if (!contains(observer.position(), observer.position())) return false;
        int nearest = index;
        double best = Double.POSITIVE_INFINITY;
        for (int i = Math.max(1, index); i < waypoints.size(); i++) {
            // Being closer to an outgoing capsule does not mean its bend has
            // been passed. Only advance() may anticipate a turn with a preview.
            if (i > index) {
                Vector3d corner = waypoints.get(i - 1);
                Vector3d incoming = new Vector3d(corner).sub(waypoints.get(i - 2));
                if (new Vector3d(observer.position()).sub(corner).dot(incoming) < 0) break;
            }
            double d = distance(observer.position(), waypoints.get(i - 1), waypoints.get(i));
            if (d < best - 1e-6) {
                best = d;
                nearest = i;
            }
        }
        Vector3d endpoint = waypoints.get(nearest);
        Vector3d anchor = closest(observer.position(), waypoints.get(Math.max(0, nearest - 1)), endpoint);
        if (!contains(observer.position(), anchor)
                || !LowAltitudePlanner.lineOfSight(world, observer.position(), endpoint, 0, 0)) return false;
        index = nearest;
        return true;
    }

    /** Skip with LOS and a safe live steering preview. After crossing a corner,
     * staged forward progress suffices; no velocity matching or input replay. */
    public boolean advance(World world, PhysicsState observer, MovementParameters movement) {
        Objects.requireNonNull(world);
        Objects.requireNonNull(observer);
        Objects.requireNonNull(movement);
        if (index + 1 >= waypoints.size() || observer.onGround()) return false;
        Vector3d next = waypoints.get(index + 1);
        if (!LowAltitudePlanner.lineOfSight(world, observer.position(), next, 0, 0)) return false;
        Vector3d current = waypoints.get(index);
        Vector3d incoming = new Vector3d(current).sub(waypoints.get(Math.max(0, index - 1)));
        FlightSafety safety = new FlightSafety(movement, DamageParameters.NONE, 20);
        // Match the controller's executable constant-heading horizon. Re-aiming
        // every preview tick certifies controls that this decision never executes.
        float yaw = LowAltitudePlanner.yaw(new Vector3d(next).sub(observer.position()));
        // A diving-only shortcut cannot advance a cruise cursor: selection
        // must offer the same controls used as the advancement proof.
        for (float pitch : VanillaGlideMode.PITCHES) {
            PhysicsState state = observer;
            for (int tick = 0; tick < 24; tick++) {
                PhysicsState before = state;
                state = LowAltitudePlanner.preview(world, state, yaw, pitch, movement, safety);
                if (state == null || geometry != null && !contains(before.position(), state.position())) break;
                // A long next segment need not be reached in one preview. Six
                // safe steering ticks making forward progress suffice after the
                // corner; subsequent controller ticks continue the rollout.
                boolean reachesCorner = new Vector3d(state.position()).sub(current).dot(incoming) >= 0;
                if (geometry != null && reachesCorner && tick == 5 && !state.onGround()
                        && state.position().distance(next) < observer.position().distance(next)) {
                    index++;
                    return true;
                }
                if (LowAltitudePlanner.nearGoal(state.position(), next)
                        && (!state.onGround() || landing && index + 1 == waypoints.size() - 1)) {
                    index++;
                    return true;
                }
                if (state.onGround()) break;
            }
        }
        return false;
    }
}
