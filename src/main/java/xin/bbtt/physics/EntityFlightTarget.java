package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.Entity.Entity;
import xin.bbtt.world.World;

/** Tick-sampled entity target. Sampling belongs to the follow movement, not steering. */
public final class EntityFlightTarget implements FlightTarget {
    private final int entityId;
    private final double keepDistance;
    private Vector3d position;
    private final Vector3d velocity = new Vector3d();
    private boolean landed;
    private boolean lost;
    private boolean gliding;

    public EntityFlightTarget(int entityId, double keepDistance, Vector3d initial) {
        this.entityId = entityId;
        this.keepDistance = keepDistance;
        this.position = new Vector3d(initial);
    }

    public void sample(World world) {
        Entity entity = world.getEntity(entityId);
        if (entity == null) { lost = true; velocity.zero(); return; }
        Vector3d next = new Vector3d(entity.getPosition());
        // EMA of tick displacements, alpha=1/5: five-tick noise filtering is an
        // engineering window, not a speed threshold. A single packet spike decays.
        velocity.lerp(new Vector3d(next).sub(position), 0.2);
        position = next;
        Object flags = entity.getMetadata().get("shared_flags");
        // Yarn 1.21.11 Entity.GLIDING_FLAG_INDEX = 7.
        gliding = flags instanceof Number number && (number.intValue() & 0x80) != 0;
        landed = !gliding && world.isOnGround(position);
    }

    public boolean gliding() { return gliding; }
    public double horizontalSpeed() { return Math.hypot(velocity.x, velocity.z); }
    @Override public Vector3d position() { return new Vector3d(position); }
    @Override public Vector3d velocity() { return new Vector3d(velocity); }
    @Override public boolean landed() { return landed; }
    @Override public boolean lost() { return lost; }
    @Override public Vector3d aim(PhysicsState observer) {
        double speed = horizontalSpeed();
        if (lost || speed == 0) return position();
        double distance = Math.hypot(position.x - observer.position().x, position.z - observer.position().z);
        double ownSpeed = Math.hypot(observer.velocity().x, observer.velocity().z);
        // Closing-time prediction: t=d/(|self|+|target|), finite even when
        // equal-speed pursuit has no intercept. Offset backwards along motion.
        double lead = distance / (ownSpeed + speed);
        return position().fma(lead, velocity).add(-velocity.x / speed * keepDistance,
                0, -velocity.z / speed * keepDistance);
    }
}
