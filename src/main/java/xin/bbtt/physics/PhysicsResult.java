package xin.bbtt.physics;

import org.joml.Vector3d;

/** Tick-end motion; collision flags also tell the caller which velocity axes to stop. */
public record PhysicsResult(
        Vector3d position, Vector3d velocity, boolean collidedX, boolean collidedZ,
        boolean onGround, double fallDistance) {
    public PhysicsResult(Vector3d position, Vector3d velocity, boolean collidedX, boolean collidedZ, boolean onGround) {
        this(position, velocity, collidedX, collidedZ, onGround, 0);
    }

    public PhysicsResult(Vector3d position, Vector3d velocity, boolean collidedX, boolean collidedZ) {
        this(position, velocity, collidedX, collidedZ, false);
    }

    public PhysicsResult {
        position = new Vector3d(position);
        velocity = new Vector3d(velocity);
    }

    @Override
    public Vector3d position() {
        return new Vector3d(position);
    }

    @Override
    public Vector3d velocity() {
        return new Vector3d(velocity);
    }
}
