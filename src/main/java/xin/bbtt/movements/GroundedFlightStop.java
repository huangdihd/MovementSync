package xin.bbtt.movements;

import xin.bbtt.MovementSync;
import xin.bbtt.physics.GlidingPhysics;
import xin.bbtt.physics.FlightHazards;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerInput;

/** Shared flight-to-ground handoff. Ground contact alone is never a completed stop. */
public final class GroundedFlightStop {
    private static final PlayerInput HOLD = new PlayerInput(false, false, false, false, false, true, false);

    private GroundedFlightStop() {}

    /**
     * Retain real sneak input, including after the movement owner has finished.
     * This does not clear the authoritative glide flag: gliding travel ignores ledge sneak,
     * so callers must keep their flight safety controller active until that flag clears.
     */
    public static void hold(MovementSync sync) {
        sync.clearInput();
        sync.setInput(HOLD);
    }

    /** Ground contact does not clear glide metadata; preview braking before applying it. */
    public static boolean steerGroundedGlide(MovementSync sync) {
        hold(sync);
        var velocity = sync.velocity.get();
        float reverse = (float) Math.toDegrees(Math.atan2(velocity.x, -velocity.z));
        sync.setYawToward(reverse);
        sync.pitch.set(0f);
        PhysicsState state = new PhysicsState(sync.position.get(), velocity, true,
                sync.fallDistance.get(), PlayerPose.GLIDING);
        var physics = new GlidingPhysics(reverse, 0, sync.hasAttachedFirework(),
                PlayerInput.NONE, sync.getMovementParameters());
        for (int tick = 0; tick < 12; tick++) {
            var next = physics.step(sync.getWorld(), state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            var p = next.position();
            if (!next.onGround() || !FlightHazards.safeLanding(sync.getWorld(), p)
                    || !sync.getWorld().isBoxColliding(p.x - PlayerPose.HALF_WIDTH, p.y - 1.0e-4,
                    p.z - PlayerPose.HALF_WIDTH, p.x + PlayerPose.HALF_WIDTH, p.y,
                    p.z + PlayerPose.HALF_WIDTH)) return false;
            state = new PhysicsState(p, next.velocity(), next.onGround(), next.fallDistance(), next.pose());
        }
        return true;
    }

    /** An observed physical stop, not a prediction of a stop on a future tick. */
    public static boolean stopped(MovementSync sync) {
        if (!sync.onGround.get() || sync.isGliding() || sync.hasAttachedFirework()) return false;
        var velocity = sync.velocity.get();
        if (velocity.x != 0 || velocity.z != 0 || velocity.y > 0) return false;
        // An observed stop needs a local contact check, not a future walking search.
        // Burning metadata alone is not a reason to keep walking on safe terrain.
        return FlightHazards.safeLanding(sync.getWorld(), sync.position.get());
    }

    /** Release a planning hold without removing the physical brake on grounded cancellation. */
    public static void release(MovementSync sync, PlayerInput ownedInput) {
        if (ownedInput == null || sync.getInput() != ownedInput) return;
        sync.clearInput();
        if (sync.onGround.get() && (sync.isGliding() || sync.velocity.get().x != 0
                || sync.velocity.get().z != 0)) hold(sync);
    }
}
