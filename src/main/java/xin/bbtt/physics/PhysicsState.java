package xin.bbtt.physics;

import org.joml.Vector3d;

/** Tick-start state; mutable vectors are copied on both input and output. */
public record PhysicsState(Vector3d position, Vector3d velocity, boolean onGround, double fallDistance, PlayerPose pose) {
    public PhysicsState(Vector3d position, Vector3d velocity, boolean onGround, double fallDistance) {
        this(position, velocity, onGround, fallDistance, PlayerPose.STANDING);
    }

    public PhysicsState(Vector3d position, Vector3d velocity, boolean onGround) {
        this(position, velocity, onGround, 0);
    }

    public PhysicsState {
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
