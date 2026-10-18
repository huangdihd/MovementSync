package xin.bbtt.flight;

import xin.bbtt.MovementSync;
import xin.bbtt.physics.FlightControlAdmission;
import xin.bbtt.physics.PlayerInput;

/** Detached diagnostic value. No world, mutable vectors, owner or callback references escape. */
public record PhysicsTickTrace(long sequence, long inputVersion, long navigationGeneration,
        FlightControlAdmission.OwnershipTrace ownership,
        double x, double y, double z, double vx, double vy, double vz,
        boolean onGround, boolean collidedX, boolean collidedZ, boolean gliding, boolean attached,
        float yaw, float pitch, PlayerInput input, double fallDistance) {
    public static PhysicsTickTrace copy(long sequence, long inputVersion, long generation,
            FlightControlAdmission.OwnershipTrace ownership, MovementSync.FlightTickTrace tick) {
        return new PhysicsTickTrace(sequence, inputVersion, generation, ownership,
                tick.position().x, tick.position().y, tick.position().z,
                tick.velocity().x, tick.velocity().y, tick.velocity().z,
                tick.onGround(), tick.collidedX(), tick.collidedZ(), tick.gliding(), tick.attached(),
                tick.yaw(), tick.pitch(), tick.input(), tick.fallDistance());
    }
}
