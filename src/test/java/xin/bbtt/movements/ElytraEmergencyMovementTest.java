package xin.bbtt.movements;

import ch.qos.logback.classic.Logger;
import ch.qos.logback.classic.spi.ILoggingEvent;
import ch.qos.logback.core.read.ListAppender;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class ElytraEmergencyMovementTest extends ElytraCommandTestSupport {
    @org.junit.jupiter.params.ParameterizedTest
    @org.junit.jupiter.params.provider.ValueSource(strings = {"minecraft:overworld", "minecraft:the_end"})
    void exhaustedRocketVoidDiversionIsEndOnly(String dimension) throws Exception {
        World voidWorld = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        };
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, voidWorld);
        movementSync.setRecoveryDimension(dimension);
        movementSync.position.set(new Vector3d(0, 60, 0));
        movementSync.velocity.set(new Vector3d(0, -.1, .5));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        Logger logger = (Logger) MovementSync.getLogger();
        ListAppender<ILoggingEvent> messages = new ListAppender<>();
        messages.start();
        logger.addAppender(messages);
        try {
            var movement = new ElytraFlyMovement(new Vector3d(0, 60, 800));
            movement.init();
            movement.onTick();
            assertFalse(movement.isFinished());
            assertEquals(dimension.equals("minecraft:the_end"), messages.list.stream()
                    .anyMatch(message -> message.getFormattedMessage().contains("Fireworks exhausted above void")));
            if (dimension.equals("minecraft:overworld")) {
                assertFalse(messages.list.stream().anyMatch(message ->
                        message.getFormattedMessage().contains("Emergency landing unavailable")));
            }
        } finally {
            logger.detachAppender(messages);
            messages.stop();
        }
    }

    @Test void exhaustedEquipmentOverVoidRetainsControlAndRetriesWithoutDisablingGliding() throws Exception {
        World voidWorld = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        };
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, voidWorld);
        inventory[6] = null;
        movementSync.inventoryManager.setContainerItems(0, inventory);
        movementSync.position.set(new Vector3d(0, 60, 0));
        movementSync.velocity.set(new Vector3d(0, -0.1, 0.5));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        Logger logger = (Logger) MovementSync.getLogger();
        ListAppender<ILoggingEvent> messages = new ListAppender<>();
        messages.start();
        logger.addAppender(messages);
        try {
            var movement = new ElytraFlyMovement(new Vector3d(0, 60, 800));
            movement.init();
            movement.onTick();
            assertFalse(movement.isFinished());
            movementSync.pitch.set(80f);
            movement.onTick();
            assertFalse(movement.isFinished());
            assertTrue(movementSync.pitch.get() <= 10);
            assertTrue(movementSync.isGliding());
            assertTrue(messages.list.stream().anyMatch(message -> message.getFormattedMessage().contains("durability low")));
            assertEquals(1, messages.list.stream().filter(message -> message.getFormattedMessage().contains("No known solid")).count());
        } finally {
            logger.detachAppender(messages);
            messages.stop();
        }
    }
}
