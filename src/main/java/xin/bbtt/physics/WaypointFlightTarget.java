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
        if (waypoints.isEmpty() || !goal.isFinite() || !Double.isFinite(corridorRadius)
                || corridorRadius < 0 || waypoints.stream().anyMatch(p -> !p.isFinite()))
            throw new IllegalArgumentException("Invalid geometric route");
        this.waypoints = waypoints.stream().map(Vector3d::new).toList();
        this.goal = new Vector3d(goal);
        this.landing = landing;
        this.partial = partial;
        this.corridorRadius = corridorRadius;
        this.geometry = geometry;
        // Planner routes include their starting point as the first corridor vertex.
        this.index = waypoints.size() > 1 ? 1 : 0;
    }

    public Vector3d goal() { return new Vector3d(goal); }
    public List<Vector3d> waypoints() { return waypoints.stream().map(Vector3d::new).toList(); }
    public int index() { return index; }
    public boolean partial() { return partial; }
    public double corridorRadius() { return corridorRadius; }
    @Override public Vector3d position() { return new Vector3d(waypoints.get(index)); }
    @Override public Vector3d velocity() { return pursuit && followed != null ? followed.velocity() : new Vector3d(); }
    @Override public boolean landed() { return !pursuit && landing && !partial && index == waypoints.size() - 1; }
    @Override public boolean lost() { return false; }

    /** A segment must fit in one convex corridor capsule (never cut across a bend).
     * Terrain must additionally be revalidated by the controller's safety prediction. */
    public boolean contains(Vector3d from, Vector3d to) {
        if (!from.isFinite() || !to.isFinite()) return false;
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
            Vector3d to = trajectory.result(tick).position();
            if (!contains(from, to)) return false;
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
        for (float pitch : new float[] {-55, -30, -10, 0, 20, 40}) {
            PhysicsState state = observer;
            for (int tick = 0; tick < 24; tick++) {
                float yaw = LowAltitudePlanner.yaw(new Vector3d(next).sub(state.position()));
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
