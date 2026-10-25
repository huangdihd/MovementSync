package xin.bbtt.flight;

import java.util.List;
import org.joml.Vector3d;
import xin.bbtt.physics.LocalAirRoute;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.physics.WaypointFlightTarget;
import xin.bbtt.world.World;

/** Initial airborne clearance only; the legacy selector still owns high cruise altitude.
 * Observations are physical ticks, not planner invocations. Missing observations cannot
 * certify clearance, and a completed handoff never re-enters climb on this leg.
 */
public final class LocalClimbHandoff {
    public enum Phase { DEPLOYMENT, LOCAL_CLIMB, CRUISE }
    public static final int STABLE_TICKS = 10;
    private static final double PHYSICAL_CLIMB_CORRIDOR = 4;
    private Phase phase = Phase.DEPLOYMENT;
    private long lastTick = Long.MIN_VALUE;
    private int stableTicks;
    private double departureY;

    public Phase phase() { return phase; }
    public int stableTicks() { return stableTicks; }

    /** An authoritative state discontinuity must prove clearance again at the new position. */
    public void resetClearance() {
        if (phase != Phase.LOCAL_CLIMB) return;
        lastTick = Long.MIN_VALUE;
        stableTicks = 0;
    }

    /** An already-gliding leg is not a fresh deployment; preserve existing cruise. */
    public void inheritAirborne() { phase = Phase.CRUISE; }

    public void deployed(double y) {
        if (phase != Phase.DEPLOYMENT) return;
        departureY = y;
        phase = Phase.LOCAL_CLIMB;
    }

    /** Ten consecutive, distinct physical observations; duplicate calls never add time. */
    public boolean observe(long tick, boolean clear) {
        return observe(tick, clear, true);
    }

    /** Replay accumulates evidence, but only its current boundary may commit ownership. */
    public boolean observe(long tick, boolean clear, boolean handoffBoundary) {
        if (phase != Phase.LOCAL_CLIMB) return phase == Phase.CRUISE;
        if (tick == lastTick) {
            if (!clear) stableTicks = 0;
            return false;
        }
        if (lastTick == Long.MIN_VALUE || tick != lastTick + 1) stableTicks = 0;
        lastTick = tick;
        stableTicks = clear ? stableTicks + 1 : 0;
        if (handoffBoundary && stableTicks >= STABLE_TICKS) phase = Phase.CRUISE;
        return phase == Phase.CRUISE;
    }

    /** Short geometric guidance, not a terminal proof or a global route-altitude scan. */
    public WaypointFlightTarget route(PhysicsState state, Vector3d goal, float yaw) {
        Vector3d p = state.position();
        Vector3d direction = new Vector3d(goal.x - p.x, 0, goal.z - p.z);
        if (direction.lengthSquared() < 1.0e-8)
            direction.set(-Math.sin(Math.toRadians(yaw)), 0, Math.cos(Math.toRadians(yaw)));
        direction.normalize().mul(24);
        Vector3d end = new Vector3d(p).add(direction);
        end.y = Math.min(World.getMaxWorldY() - PlayerPose.GLIDING.height() - 2,
                Math.max(departureY + 16, p.y + 8));
        return new WaypointFlightTarget(List.of(p, end), end, false, false,
                PHYSICAL_CLIMB_CORRIDOR);
    }

    /** Height is relative to local collision terrain/departure, never a fixed goal Y.
     * The 16-block local clearance matches cruise's terrain margin without invoking
     * its distant/unknown-terrain altitude policy (which remains unchanged).
     */
    public boolean clear(World world, PhysicsState state, WaypointFlightTarget route) {
        Vector3d p = state.position(), end = route.goal();
        Vector3d forward = new Vector3d(end.x, p.y, end.z);
        Vector3d lower = new Vector3d(p).add(0, -16, 0);
        Vector3d lowerForward = new Vector3d(forward).add(0, -16, 0);
        return !state.onGround() && p.y >= departureY + 8
                && state.velocity().y >= -.15
                && Math.hypot(state.velocity().x, state.velocity().z) >= .65
                && LocalAirRoute.visible(world, p, end, PlayerPose.GLIDING)
                && LocalAirRoute.visible(world, lower, lowerForward, PlayerPose.GLIDING)
                && LocalAirRoute.visible(world, p, lower, PlayerPose.GLIDING)
                && LocalAirRoute.visible(world, forward, lowerForward, PlayerPose.GLIDING);
    }
}
