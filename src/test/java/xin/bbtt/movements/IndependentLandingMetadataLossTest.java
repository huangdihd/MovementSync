package xin.bbtt.movements;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/** Observed coordinates are a seed; this loaded flat shore is a synthetic fixture. */
class IndependentLandingMetadataLossTest extends ElytraCommandTestSupport {
    private World install() throws Exception {
        var stone = new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE");
        var air = new BlockState("air", 0, Map.of(), "empty", 0, false, "AIR");
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 33 ? stone : air; }
            @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
                return y < 33;
            }
        };
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.movementController.pause();
        movementSync.position.set(new Vector3d(245.5, 34.25, 20.5));
        movementSync.velocity.set(new Vector3d(.08, -.07544, 0));
        movementSync.yaw.set(-95.4657f);
        movementSync.pitch.set(0f);
        movementSync.onGround.set(false);
        return world;
    }

    @Test void acceptedLandingRetainsOwnerAfterMetadataClearsUntilSupportedStop() throws Exception {
        exercise(false);
    }

    @Test void metadataLostBeforeIndependentInitDoesNotRestartOrAbandonFlight() throws Exception {
        exercise(true);
    }

    @Test void stopNeverClearsAnAirborneGlide() throws Exception {
        install();
        movementSync.setGliding(true);
        var owner = new ElytraFlyMovement(new Vector3d(245.5, 33, 20.5)).forIndependentLanding();
        owner.init();
        owner.onStop();
        assertTrue(movementSync.isGliding());
        assertFalse(movementSync.onGround.get());
        assertEquals(new Vector3d(.08, -.07544, 0), movementSync.velocity.get());
    }

    @Test void missingNonglidingProofKeepsSafetyOwnerRatherThanTimingOut() throws Exception {
        movementSync.movementController.pause();
        movementSync.position.set(new Vector3d(245.5, 34.25, 20.5));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        movementSync.cancelNavigation();
        var owner = assertInstanceOf(ElytraFlyMovement.class,
                movementSync.movementController.getMovements().get(0));
        owner.init();
        movementSync.setGliding(false);
        for (int tick = 0; tick < 125; tick++) owner.onTick();
        assertFalse(owner.isFinished());
        assertFalse(movementSync.isGliding());
        assertNotEquals(PlayerInput.NONE, movementSync.getInput());
    }

    private void exercise(boolean beforeInit) throws Exception {
        World world = install();
        movementSync.setGliding(true);
        movementSync.cancelNavigation();
        var owner = assertInstanceOf(ElytraFlyMovement.class,
                movementSync.movementController.getMovements().get(0));
        if (!beforeInit) owner.init();
        movementSync.setGliding(false); // Simulated authoritative metadata, never a controller action.
        if (beforeInit) owner.init();
        owner.onTick();
        assertFalse(owner.isFinished(), "metadata loss is not touchdown");
        assertFalse(movementSync.isGliding(), "safety must not fabricate glide metadata");
        assertNotEquals(PlayerInput.NONE, movementSync.getInput(), "nongliding body retains physical controls");
        for (int tick = 0; tick < 100 && !owner.isFinished(); tick++) {
            var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                    movementSync.onGround.get(), movementSync.fallDistance.get());
            var next = new WalkingPhysics(movementSync.snapshotInput(), movementSync.yaw.get(), true,
                    movementSync.getMovementParameters()).step(world, state, PlayerPose.HALF_WIDTH, (double) 1.8f);
            movementSync.position.set(next.position());
            movementSync.velocity.set(next.velocity());
            movementSync.onGround.set(next.onGround());
            movementSync.fallDistance.set(next.fallDistance());
            owner.onTick();
            if (owner.isFinished()) assertTrue(GroundedFlightStop.stopped(movementSync));
        }
        assertTrue(owner.isFinished(), "finite supported stop");
        assertEquals(PlayerInput.NONE, movementSync.getInput());
        assertFalse(movementSync.isGliding());
    }
}
