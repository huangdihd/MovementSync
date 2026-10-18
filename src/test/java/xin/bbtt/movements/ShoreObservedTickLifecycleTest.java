package xin.bbtt.movements;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.FlightHazards;
import xin.bbtt.tasks.updateMotionTask;
import xin.bbtt.world.World;

import java.lang.reflect.Method;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/** Numeric shore trace through the real input/physics/packet tick, not a preview. */
class ShoreObservedTickLifecycleTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final BlockState LAVA = new BlockState("lava", 0, Map.of("level", "0"), "empty", 0, false, "LAVA");

    private void install() throws Exception {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.x < 246 && p.y < 33) return STONE;
                return p.y < 31 ? LAVA : AIR;
            }
        };
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.position.set(new Vector3d(245.5, 33, 20.5));
        movementSync.velocity.set(new Vector3d(0, -.0784, 0));
        movementSync.onGround.set(true);
        movementSync.yaw.set(-95.4657f);
        movementSync.pitch.set(0f);
    }

    private static void tick(updateMotionTask task) throws Exception {
        Method method = updateMotionTask.class.getDeclaredMethod("tick");
        method.setAccessible(true);
        method.invoke(task);
    }

    @Test void fiveObservedPostJumpTicksReachLoggedDeploymentApex() throws Exception {
        install();
        var task = new updateMotionTask();
        movementSync.jump(); // action 258
        tick(task); // observed physics 259
        assertEquals(33.42, movementSync.position.get().y, 1e-6);
        assertEquals(.3332, movementSync.velocity.get().y, 1e-6);
        for (int i = 0; i < 5; i++) tick(task);
        assertEquals(34.2522, movementSync.position.get().y, 1e-4);
        assertEquals(-.07544, movementSync.velocity.get().y, 1e-5);
        assertTrue(movementSync.startGliding());
        assertTrue(movementSync.isGliding(), "prediction is not server acceptance");
        for (int i = 0; i < 5; i++) {
            tick(task);
            assertTrue(movementSync.position.get().y > 33);
            assertTrue(movementSync.position.get().x < 246);
        }
        movementSync.applySharedFlags((byte) 0); // explicit rejected/interrupted metadata branch
        GroundedFlightStop.hold(movementSync);
        for (int i = 0; i < 80; i++) {
            tick(task);
            assertTrue(movementSync.position.get().y >= 33);
            assertTrue(FlightHazards.safeLanding(movementSync.getWorld(), movementSync.position.get()));
        }
        assertTrue(GroundedFlightStop.stopped(movementSync));
    }

    @Test void acceptedMetadataBranchCanRetainGroundBrakeWithoutInventingGlideClearance() throws Exception {
        install();
        var task = new updateMotionTask();
        movementSync.jump();
        for (int i = 0; i < 6; i++) tick(task);
        assertTrue(movementSync.startGliding());
        for (int i = 0; i < 5; i++) tick(task);
        movementSync.applySharedFlags((byte) 0x80); // accepted branch is supplied, never inferred
        GroundedFlightStop.hold(movementSync);
        movementSync.yaw.set(84.5343f);
        for (int i = 0; i < 80; i++) {
            tick(task);
            assertTrue(movementSync.position.get().y >= 33);
            assertTrue(movementSync.position.get().x < 246.3);
            assertTrue(movementSync.isGliding());
        }
        assertTrue(movementSync.onGround.get());
        assertFalse(GroundedFlightStop.stopped(movementSync));
        movementSync.applySharedFlags((byte) 0);
        for (int i = 0; i < 40; i++) tick(task);
        assertTrue(GroundedFlightStop.stopped(movementSync));
    }

    @Test void observedJumpThenCancellationKeepsRealControllerThroughBothMetadataBranches() throws Exception {
        for (boolean accepted : new boolean[]{false, true}) {
            install();
            movementSync.setGliding(false);
            movementSync.movementController.pause();
            var task = new updateMotionTask();
            movementSync.jump();
            for (int i = 0; i < 6; i++) tick(task);
            assertEquals(34.2522, movementSync.position.get().y, 1e-4);
            assertTrue(movementSync.startGliding());
            for (int i = 0; i < 5; i++) tick(task);
            // Cancellation starts with the deployed physics state, not fabricated motion.
            movementSync.applySharedFlags((byte) 0x80);
            movementSync.cancelNavigation();
            var owner = assertInstanceOf(ElytraFlyMovement.class,
                    movementSync.movementController.getMovements().get(0));
            owner.init();
            if (!accepted) movementSync.applySharedFlags((byte) 0);
            for (int i = 0; i < 120; i++) {
                owner.onTick();
                tick(task);
                assertTrue(movementSync.position.get().y >= 33, "must stay above the lava coast");
                assertTrue(FlightHazards.safeLanding(movementSync.getWorld(), movementSync.position.get()));
                if (accepted && movementSync.onGround.get()) movementSync.applySharedFlags((byte) 0);
                if (owner.isFinished()) break;
            }
            assertTrue(owner.isFinished());
            assertTrue(GroundedFlightStop.stopped(movementSync));
            assertTrue(movementSync.position.get().x < 246.3);
            owner.onStop();
        }
    }

    @Test void inheritedMidairGlideLossReturnsToTakeoffRatherThanStaleAirborneState() throws Exception {
        install();
        movementSync.position.set(new Vector3d(245.5, 34.25, 20.5));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        var lifecycle = new VanillaGlideLifecycle();
        lifecycle.init(movementSync);
        movementSync.applySharedFlags((byte) 0);
        assertTrue(lifecycle.awaitingTakeoff(movementSync));
    }

    @Test void groundedClientRetainsGlideUntilMetadataAndDoesNotInventAcknowledgement() throws Exception {
        install();
        movementSync.setGliding(true);
        var task = new updateMotionTask();
        GroundedFlightStop.hold(movementSync);
        tick(task);
        assertTrue(movementSync.onGround.get());
        assertTrue(movementSync.isGliding(), "official 1.21.11 updateFallFlying is server gated");
        assertTrue(movementSync.position.get().x > 245.5, "grounded glide still accelerates despite sneak");
        assertFalse(GroundedFlightStop.stopped(movementSync));
        movementSync.applySharedFlags((byte) 0);
        tick(task);
        assertFalse(movementSync.isGliding());
    }
}
