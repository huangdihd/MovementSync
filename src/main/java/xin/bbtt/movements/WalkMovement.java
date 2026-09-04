package xin.bbtt.movements;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;

public class WalkMovement extends Movement {
    private static final double VELOCITY_EPSILON = 1.0e-9;

    private final Vector3d velocity;
    private final long time;
    private volatile Vector3d velocityBeforeStart;
    private volatile Vector3d expectedVelocity;

    public WalkMovement(Vector3d velocity, long time){
        this.velocity = new Vector3d(velocity);
        this.time = time;
    }

    @Override
    public void init() {
        MovementSync.INSTANCE.velocity.updateAndGet(current -> {
            velocityBeforeStart = new Vector3d(current);
            expectedVelocity = new Vector3d(current).add(velocity);
            return new Vector3d(expectedVelocity);
        });
    }

    @Override
    public void onTick() {

    }

    @Override
    public long getTime() {
        return time;
    }

    @Override
    public void onStop() {
        Vector3d before = velocityBeforeStart;
        Vector3d expected = expectedVelocity;
        if (before == null || expected == null) return;
        MovementSync.INSTANCE.velocity.updateAndGet(current -> new Vector3d(
            unchanged(current.x, expected.x) ? before.x : current.x,
            unchanged(current.y, expected.y) ? before.y : current.y,
            unchanged(current.z, expected.z) ? before.z : current.z
        ));
    }

    private static boolean unchanged(double current, double expected) {
        return Math.abs(current - expected) <= VELOCITY_EPSILON;
    }
}
