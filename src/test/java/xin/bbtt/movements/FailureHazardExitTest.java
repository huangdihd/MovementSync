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

class FailureHazardExitTest extends ElytraCommandTestSupport {
    private World install(String hazard, int cell) throws Exception {
        var stone = new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE");
        var air = new BlockState("air", 0, Map.of(), "empty", 0, false, "AIR");
        var fire = new BlockState(hazard, 0, Map.of("level", "0"), "empty", 0, false, "AIR");
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y < 1) return stone;
                return Math.floor(p.x) == cell && Math.floor(p.z) == 0 && p.y < 2 ? fire : air;
            }
            @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
                return y < 1;
            }
        };
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.position.set(new Vector3d(cell + .5, 1, .5));
        movementSync.onGround.set(true);
        return world;
    }

    @Test void positiveAndNegativeBoundaryContactIsNotSafe() throws Exception {
        for (int cell : new int[] {-2, 2}) {
            World world = install("fire", cell);
            assertFalse(FlightHazards.safeLanding(world, new Vector3d(cell - PlayerPose.HALF_WIDTH, 1, .5)));
            assertFalse(FlightHazards.safeLanding(world, new Vector3d(cell + 1 + PlayerPose.HALF_WIDTH, 1, .5)));
            assertTrue(FlightHazards.safeLanding(world, new Vector3d(cell - PlayerPose.HALF_WIDTH - .001, 1, .5)));
        }
    }

    @Test void actualWalkingLeavesFireAndSoulFireAndStopsWithoutOldGoal() throws Exception {
        for (String hazard : new String[] {"fire", "soul_fire", "lava"}) {
            World world = install(hazard, -2);
            movementSync.velocity.set(new Vector3d());
            var exit = new HazardExitMovement(movementSync.getNavigationGeneration());
            for (int tick = 0; tick < 121 && !exit.isFinished(); tick++) {
                exit.onTick();
                var before = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                        movementSync.onGround.get(), movementSync.fallDistance.get());
                var next = new WalkingPhysics(movementSync.snapshotInput(), movementSync.yaw.get(), true,
                        movementSync.getMovementParameters()).step(world, before, PlayerPose.HALF_WIDTH, (double) 1.8f);
                movementSync.position.set(next.position());
                movementSync.velocity.set(next.velocity());
                movementSync.onGround.set(next.onGround());
                movementSync.fallDistance.set(next.fallDistance());
            }
            assertTrue(exit.isFinished(), hazard);
            assertTrue(GroundedFlightStop.stopped(movementSync), hazard);
            assertEquals(PlayerInput.NONE, movementSync.getInput());
        }
    }

    @Test void failureClearsOldKeysWithoutQueueingSafetySteering() throws Exception {
        install("fire", 2);
        movementSync.movementController.pause();
        var stale = new PlayerInput(true, false, true, false, true, false, true);
        movementSync.setInput(stale);
        movementSync.abandonNavigationRequest(movementSync.getNavigationGeneration(), "UNSAFE_ROUTE");
        assertEquals(PlayerInput.NONE, movementSync.snapshotInput());
        assertTrue(movementSync.movementController.getMovements().isEmpty(),
                "terminal failure must leave natural physics in control");
    }

    @Test void explicitCancelStillQueuesIndependentSafetyOwner() throws Exception {
        install("fire", 2);
        movementSync.movementController.pause();
        var stale = new PlayerInput(true, false, true, false, true, false, true);
        movementSync.setInput(stale);
        movementSync.cancelNavigation();
        assertEquals(PlayerInput.NONE, movementSync.snapshotInput());
        assertEquals(1, movementSync.movementController.getMovements().size());
        assertInstanceOf(HazardExitMovement.class, movementSync.movementController.getMovements().get(0));
        movementSync.onPlayerDeath();
        movementSync.cancelNavigation();
        assertTrue(movementSync.movementController.getMovements().isEmpty());
        assertEquals(PlayerInput.NONE, movementSync.snapshotInput());
    }

    @Test void burningOnSafeGroundDoesNotWalkForever() throws Exception {
        install("air", 2);
        movementSync.getSelfMetadata().put("shared_flags", (byte) 1);
        assertFalse(HazardExitMovement.needed(movementSync), "residual burning alone must not restart movement on safe ground");
        var exit = new HazardExitMovement(movementSync.getNavigationGeneration());
        exit.onTick();
        assertTrue(exit.isFinished(), "walking cannot extinguish residual burning on safe terrain");
        assertEquals(PlayerInput.NONE, movementSync.snapshotInput());
    }

    @Test void unknownIdleSupportDoesNotStartHazardExit() {
        movementSync.onGround.set(true);
        movementSync.velocity.set(new Vector3d());
        assertFalse(HazardExitMovement.needed(movementSync));
    }

    @Test void staleSafetyTickCannotOverwriteNewOwner() throws Exception {
        install("fire", 2);
        var exit = new HazardExitMovement(movementSync.getNavigationGeneration());
        movementSync.beginNavigationRequest();
        var fresh = new PlayerInput(false, true, false, false, false, false, false);
        movementSync.setInput(fresh);
        exit.onTick();
        exit.onStop();
        assertSame(fresh, movementSync.getInput());
    }
}
