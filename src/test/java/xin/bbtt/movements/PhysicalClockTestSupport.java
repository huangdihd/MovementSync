package xin.bbtt.movements;

import org.joml.Vector3d;
import org.junit.jupiter.api.BeforeEach;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.GlidingPhysics;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PhysicsStepper;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

/** Manual driver: only a simulated and committed physics step advances the clock. */
abstract class PhysicalClockTestSupport extends ElytraCommandTestSupport {
    @BeforeEach
    void installAirborneWorld() throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        });
        movementSync.position.set(new Vector3d(0, 1000, 0));
        movementSync.velocity.set(new Vector3d(0, 0, .3));
        movementSync.onGround.set(false);
    }

    protected void physicalTick() {
        var input = movementSync.snapshotInput();
        boolean gliding = movementSync.isGliding();
        var pose = gliding ? PlayerPose.GLIDING : PlayerPose.STANDING;
        PhysicsStepper physics = gliding
                ? new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(), false,
                        input, movementSync.getMovementParameters())
                : new WalkingPhysics(input, movementSync.yaw.get(), true,
                        movementSync.getMovementParameters());
        var result = physics.step(movementSync.getWorld(),
                new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                        movementSync.onGround.get(), movementSync.fallDistance.get(), pose),
                PlayerPose.HALF_WIDTH, pose.height());
        movementSync.position.set(result.position());
        movementSync.velocity.set(result.velocity());
        movementSync.onGround.set(result.onGround());
        movementSync.fallDistance.set(result.fallDistance());
        movementSync.completePhysicsStep();
    }
}
