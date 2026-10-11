package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerState;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.movement.MovementController;
import xin.bbtt.tasks.MovementTask;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

final class ElytraTakeoffMovementTest extends ElytraCommandTestSupport {
    @Test
    void groundJumpWaitsThroughAscentThenRequestsGlidingExactlyOnce() {
        ElytraTakeoffMovement movement = new ElytraTakeoffMovement();
        movementSync.velocity.set(new Vector3d(0.12, 0, -0.2));

        movement.init();
        assertTrue(movementSync.onGround.get());
        assertEquals(new Vector3d(0.12, 0, -0.2), movementSync.velocity.get());
        PlayerInput input = movementSync.snapshotInput();
        assertTrue(input.jump());
        assertFalse(movementSync.snapshotInput().jump(), "jump must be a one-shot request");
        double startY = movementSync.position.get().y;
        stepPhysics(input);
        assertFalse(movementSync.onGround.get());
        assertEquals(startY + 0.42, movementSync.position.get().y, 1.0e-9);
        assertEquals((0.42 - 0.08) * 0.9800000190734863D, movementSync.velocity.get().y, 1.0e-9);
        assertEquals(0.12 * 0.6 * 0.91, movementSync.velocity.get().x, 1.0e-9);
        assertEquals(-0.2 * 0.6 * 0.91, movementSync.velocity.get().z, 1.0e-9);
        movement.onTick();
        assertFalse(movement.isFinished());
        assertTrue(packets.isEmpty());
        assertNull(movementSync.pollBeforeMovementPacket());

        for (int tick = 0; tick < 10 && movementSync.velocity.get().y > 0; tick++) {
            movement.onTick();
            assertFalse(movement.isFinished());
            assertNull(movementSync.pollBeforeMovementPacket());
            stepPhysics(movementSync.snapshotInput());
        }
        assertTrue(movementSync.velocity.get().y < 0, "physics must reach the descending phase");
        movement.onTick();
        assertTrue(movement.isFinished());
        assertTrue(movementSync.isGliding());
        assertStartPacket();
        movement.onTick();
        movement.onStop();
        assertTrue(packets.isEmpty(), "gliding commands must wait for the motion tick");
        assertNull(movementSync.pollBeforeMovementPacket(), "takeoff must not enqueue a duplicate command");
        assertTrue(movementSync.isGliding(), "finishing the takeoff must not stop flight");
    }

    @Test
    void alreadyDescendingDoesNotJumpAndStartsImmediately() {
        movementSync.onGround.set(false);
        movementSync.velocity.set(new Vector3d(0, -0.2, 0));
        ElytraTakeoffMovement movement = new ElytraTakeoffMovement();
        movement.init();
        assertEquals(-0.2, movementSync.velocity.get().y);
        assertFalse(movementSync.snapshotInput().jump());
        movement.onTick();
        assertTrue(movement.isFinished());
        assertStartPacket();
    }

    @Test
    void zeroVerticalVelocityIsEligibleOnlyWhenAirborne() {
        movementSync.onGround.set(false);
        ElytraTakeoffMovement movement = new ElytraTakeoffMovement();
        movement.init();
        movementSync.onGround.set(true);
        movement.onTick();
        assertFalse(movement.isFinished());
        assertTrue(packets.isEmpty());
        assertNull(movementSync.pollBeforeMovementPacket());
        movementSync.onGround.set(false);
        movement.onTick();
        assertTrue(movement.isFinished());
        assertStartPacket();
    }

    @Test
    void rejectedStartFinishesWithoutRetryingAfterEquipmentReturns() {
        movementSync.onGround.set(false);
        movementSync.velocity.set(new Vector3d(0, -0.2, 0));
        inventory[6] = null;
        ElytraTakeoffMovement movement = new ElytraTakeoffMovement();
        movement.init();
        movement.onTick();
        assertTrue(movement.isFinished());
        assertFalse(movementSync.isGliding());
        assertTrue(packets.isEmpty());
        assertNull(movementSync.pollBeforeMovementPacket());
        inventory[6] = item("elytra", 1);
        movement.onTick();
        assertTrue(packets.isEmpty(), "a failed attempt must not retry indefinitely");
        assertNull(movementSync.pollBeforeMovementPacket());
    }

    @Test
    void taskTimesOutAtSixtyTicksAndControllerStopsTheMovement() throws Exception {
        ElytraTakeoffMovement movement = new ElytraTakeoffMovement();
        MovementController controller = movementSync.getMovementController();
        // Install an activation without scheduling: timeout uses logical ticks, not wall time.
        Field currentMovement = MovementController.class.getDeclaredField("currentMovement");
        currentMovement.setAccessible(true);
        currentMovement.set(controller, movement);
        movement.init();
        MovementTask task = new MovementTask(movement, controller, 0L);
        assertEquals(3000L, movement.getTime());
        for (int tick = 0; tick < 59; tick++) task.run();
        assertSame(movement, controller.getCurrentMovement());
        assertFalse(movement.isFinished());
        task.run();
        assertNull(controller.getCurrentMovement());
        assertTrue(movement.isFinished(), "controller timeout must invoke onStop");
        assertFalse(movementSync.isGliding());
        assertTrue(packets.isEmpty());
        assertNull(movementSync.pollBeforeMovementPacket());
        movementSync.velocity.set(new Vector3d(0, -0.1, 0));
        movement.onTick();
        assertTrue(packets.isEmpty(), "timed-out takeoff must remain stopped");
        assertNull(movementSync.pollBeforeMovementPacket());
    }

    @Test
    void cancellationFinishesWithoutSendingOrStoppingFlight() {
        ElytraTakeoffMovement movement = new ElytraTakeoffMovement();
        movement.init();
        movement.onStop();
        movementSync.velocity.set(new Vector3d(0, -0.1, 0));
        movement.onTick();
        assertTrue(movement.isFinished());
        assertTrue(packets.isEmpty());
        assertNull(movementSync.pollBeforeMovementPacket());
    }

    private void stepPhysics(PlayerInput input) {
        World world = new World() {
            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return false;
            }
        };
        PhysicsResult result = new WalkingPhysics(input, movementSync.yaw.get()).step(
                world, new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                        movementSync.onGround.get()), 0.299, 1.8);
        movementSync.position.set(result.position());
        movementSync.velocity.set(result.velocity());
        movementSync.onGround.set(result.onGround());
    }

    private void assertStartPacket() {
        assertTrue(packets.isEmpty(), "gliding commands must wait for the motion tick");
        ServerboundPlayerCommandPacket packet = assertInstanceOf(
                ServerboundPlayerCommandPacket.class, movementSync.pollBeforeMovementPacket());
        assertNull(movementSync.pollBeforeMovementPacket(), "exactly one command must be queued");
        assertEquals(PlayerState.START_ELYTRA_FLYING, packet.getState());
        assertEquals(42, packet.getEntityId());
    }
}
