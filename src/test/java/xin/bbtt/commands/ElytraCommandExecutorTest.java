package xin.bbtt.commands;

import static org.junit.jupiter.api.Assertions.*;

import ch.qos.logback.classic.Level;
import ch.qos.logback.classic.Logger;
import ch.qos.logback.classic.spi.ILoggingEvent;
import ch.qos.logback.core.read.ListAppender;
import java.util.List;
import java.util.Set;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.movements.ElytraTakeoffMovement;

final class ElytraCommandExecutorTest extends ElytraCommandTestSupport {
    private final ElytraCommandExecutor executor = new ElytraCommandExecutor();
    private final Command command = new Command("elytra", new String[0], "Elytra", "elytra <start|boost|status>");
    private final ListAppender<ILoggingEvent> messages = new ListAppender<>();
    private Logger logger;
    private Level previousLevel;

    @BeforeEach
    void pauseAndCaptureMessages() {
        movementSync.getMovementController().pause();
        logger = (Logger) MovementSync.getLogger();
        previousLevel = logger.getLevel();
        logger.setLevel(Level.INFO);
        messages.start();
        logger.addAppender(messages);
    }

    @AfterEach
    void restoreLogger() {
        if (logger == null) return;
        logger.detachAppender(messages);
        logger.setLevel(previousLevel);
        messages.stop();
    }

    @Test
    void missingChestElytraRejectsStartEvenWhenHeld() {
        inventory[36] = inventory[6];
        inventory[6] = null;
        assertRejectedStart();
    }

    @Test
    void wrongChestItemRejectsStart() {
        inventory[6] = item("stone", 1);
        assertRejectedStart();
    }

    @Test
    void emptyElytraStackRejectsStart() {
        inventory[6] = item("elytra", 0);
        assertRejectedStart();
    }

    @Test
    void ridingRejectsStart() {
        movementSync.setVehicleId(99);
        assertRejectedStart();
    }

    @Test
    void alreadyGlidingRejectsStartWithoutStoppingFlight() {
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        assertRejectedStart();
        assertTrue(movementSync.isGliding());
    }

    @Test
    void validStartQueuesTakeoffWithoutJumpingOrSendingWhilePaused() {
        execute("start");
        var controller = movementSync.getMovementController();
        assertEquals(1, controller.getMovements().size());
        assertInstanceOf(ElytraTakeoffMovement.class, controller.getMovements().get(0));
        assertNull(controller.getCurrentMovement());
        assertTrue(controller.isPaused());
        assertTrue(movementSync.onGround.get());
        assertEquals(new Vector3d(), movementSync.velocity.get());
        assertTrue(packets.isEmpty());
    }

    @Test
    void boostRequestsHeldRocketWithoutInventingAttachmentOrConsumingInventory() {
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        inventory[36] = item("firework_rocket", 3);
        execute("boost");
        assertTrue(packets.isEmpty());
        ServerboundUseItemPacket packet = assertInstanceOf(ServerboundUseItemPacket.class, movementSync.pollBeforeMovementPacket());
        assertEquals(Hand.MAIN_HAND, packet.getHand());
        assertEquals(3, inventory[36].getAmount());
        assertFalse(movementSync.hasAttachedFirework());
        assertTrue(movementSync.getMovementController().getMovements().isEmpty());
    }

    @Test
    void boostWithoutGlidingReportsFailureAndSendsNothing() {
        inventory[36] = item("firework_rocket", 3);
        assertRejectedBoost();
    }

    @Test
    void boostWithoutRocketReportsFailureAndPreservesFlight() {
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        assertRejectedBoost();
        assertTrue(movementSync.isGliding());
    }

    @Test
    void statusReportsStateWithoutChangingFlightMotionInventoryOrQueue() {
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        Vector3d position = new Vector3d(1.5, 80, -4.5);
        Vector3d velocity = new Vector3d(0.3, -0.1, 0.4);
        movementSync.position.set(position);
        movementSync.velocity.set(velocity);
        movementSync.yaw.set(90f);
        movementSync.pitch.set(-12f);
        inventory[36] = item("firework_rocket", 3);
        ElytraTakeoffMovement queued = new ElytraTakeoffMovement();
        movementSync.getMovementController().addMovement(queued);
        long generation = movementSync.getNavigationGeneration();

        execute("status");

        assertTrue(movementSync.isGliding());
        assertFalse(movementSync.onGround.get());
        assertSame(position, movementSync.position.get());
        assertEquals(new Vector3d(1.5, 80, -4.5), position);
        assertSame(velocity, movementSync.velocity.get());
        assertEquals(new Vector3d(0.3, -0.1, 0.4), velocity);
        assertEquals(90f, movementSync.yaw.get());
        assertEquals(-12f, movementSync.pitch.get());
        assertEquals(generation, movementSync.getNavigationGeneration());
        assertEquals(List.of(queued), movementSync.getMovementController().getMovements());
        assertTrue(movementSync.getMovementController().isPaused());
        assertEquals(3, inventory[36].getAmount());
        assertFalse(movementSync.hasAttachedFirework());
        assertTrue(packets.isEmpty());
        assertLogged(LangManager.get("movementsync.command.elytra.status", true,
                1.5, 80.0, -4.5, 0.3, -0.1, 0.4, 0.5, 90f, -12f, false));
    }

    @Test
    void exposesFlightCommandsNotAProtocolStopToggle() {
        assertEquals(Set.of("start", "boost", "status", "fly"), executor.getSubCommands().keySet());
    }

    private void assertRejectedStart() {
        Vector3d velocity = new Vector3d(movementSync.velocity.get());
        execute("start");
        assertTrue(movementSync.getMovementController().getMovements().isEmpty());
        assertNull(movementSync.getMovementController().getCurrentMovement());
        assertEquals(velocity, movementSync.velocity.get());
        assertTrue(packets.isEmpty());
        assertFalse(messages.list.isEmpty(), "rejection should explain why no action was queued");
    }

    private void assertRejectedBoost() {
        execute("boost");
        assertTrue(packets.isEmpty());
        assertTrue(movementSync.getMovementController().getMovements().isEmpty());
        assertLogged(LangManager.get("movementsync.command.elytra.boost.failed"));
    }

    private void assertLogged(String expected) {
        assertTrue(messages.list.stream().anyMatch(message -> expected.equals(message.getFormattedMessage())),
                () -> "Expected log message: " + expected);
    }

    private void execute(String subcommand) {
        executor.onCommand(command, "elytra", new String[]{subcommand});
    }
}
