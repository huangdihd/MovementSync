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

final class ElytraTakeoffMovementTest extends ElytraCommandTestSupport {
    @Test
    void groundJumpWaitsThroughAscentThenRequestsGlidingExactlyOnce() {
        ElytraTakeoffMovement movement = new ElytraTakeoffMovement();
        movementSync.velocity.set(new Vector3d(0.12, 0, -0.2));

        movement.init();
        assertFalse(movementSync.onGround.get());
        assertEquals(new Vector3d(0.12, 0.42, -0.2), movementSync.velocity.get());
        movement.onTick();
        assertFalse(movement.isFinished());
        assertTrue(packets.isEmpty());

        movementSync.velocity.set(new Vector3d(0.12, -0.01, -0.2));
        movement.onTick();
        assertTrue(movement.isFinished());
        assertTrue(movementSync.isGliding());
        assertStartPacket();
        movement.onTick();
        movement.onStop();
        assertEquals(1, packets.size());
        assertTrue(movementSync.isGliding(), "finishing the takeoff must not stop flight");
    }

    @Test
    void alreadyDescendingDoesNotJumpAndStartsImmediately() {
        movementSync.onGround.set(false);
        movementSync.velocity.set(new Vector3d(0, -0.2, 0));
        ElytraTakeoffMovement movement = new ElytraTakeoffMovement();
        movement.init();
        assertEquals(-0.2, movementSync.velocity.get().y);
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
        inventory[6] = item("elytra", 1);
        movement.onTick();
        assertTrue(packets.isEmpty(), "a failed attempt must not retry indefinitely");
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
        movementSync.velocity.set(new Vector3d(0, -0.1, 0));
        movement.onTick();
        assertTrue(packets.isEmpty(), "timed-out takeoff must remain stopped");
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
    }

    private void assertStartPacket() {
        assertEquals(1, packets.size());
        ServerboundPlayerCommandPacket packet = assertInstanceOf(ServerboundPlayerCommandPacket.class, packets.get(0));
        assertEquals(PlayerState.START_ELYTRA_FLYING, packet.getState());
        assertEquals(42, packet.getEntityId());
    }
}
