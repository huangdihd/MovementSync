package xin.bbtt.movements;

import ch.qos.logback.classic.Level;
import ch.qos.logback.classic.Logger;
import ch.qos.logback.classic.spi.ILoggingEvent;
import ch.qos.logback.core.read.ListAppender;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.LangManager;

import static org.junit.jupiter.api.Assertions.*;

final class ElytraFlyMovementTest extends ElytraCommandTestSupport {
    private final ListAppender<ILoggingEvent> messages = new ListAppender<>();
    private Logger logger;
    private Level previousLevel;

    @BeforeEach
    void captureMessages() {
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
    void replacementAndStopPreserveFlightAndAttitude() {
        movementSync.setGliding(true);
        movementSync.onGround.set(false);
        ElytraFlyMovement flight = new ElytraFlyMovement(new Vector3d(400, 0, 0));
        movementSync.beginStaticNavigationRequest(null);
        movementSync.yaw.set(71f);
        movementSync.pitch.set(-12f);
        flight.onTick();
        assertTrue(flight.isFinished());
        assertEquals(71f, movementSync.yaw.get());
        assertEquals(-12f, movementSync.pitch.get());
        assertTrue(movementSync.isGliding());
        ElytraFlyMovement stopped = new ElytraFlyMovement(new Vector3d(400, 0, 0));
        stopped.onStop();
        stopped.onTick();
        assertEquals(71f, movementSync.yaw.get());
        assertTrue(movementSync.isGliding());
    }

    @Test
    void takeoffTimeoutFinishesWithoutSteering() {
        ElytraFlyMovement flight = new ElytraFlyMovement(new Vector3d(400, 0, 0));
        flight.init();
        assertTrue(movementSync.snapshotInput().jump());
        for (int tick = 0; tick < 61; tick++) flight.onTick();
        assertTrue(flight.isFinished());
        assertFalse(movementSync.isGliding());
    }

    @ParameterizedTest
    @ValueSource(doubles = {10, 16, 40})
    void landingBetweenDeploymentAndNextActionTickFinishesImmediately(double distance) {
        movementSync.onGround.set(false);
        movementSync.position.set(new Vector3d(0, 1, 0));
        ElytraFlyMovement flight = new ElytraFlyMovement(new Vector3d(0, 0, distance));
        flight.onTick();
        assertTrue(movementSync.isGliding());
        movementSync.setGliding(false);
        movementSync.onGround.set(true);
        flight.onTick();
        assertTrue(flight.isFinished(), "landing must not restart the takeoff timeout");
        assertLandingMessage(distance);
        int messageCount = messages.list.size();
        flight.onTick();
        assertEquals(messageCount, messages.list.size(), "landing must be settled only once");
    }

    @ParameterizedTest
    @ValueSource(doubles = {10, 16, 40})
    void landingBetweenGlideHandoffAndFirstActionTickFinishesImmediately(double distance) {
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        movementSync.position.set(new Vector3d(0, 1, 0));
        ElytraFlyMovement flight = new ElytraFlyMovement(new Vector3d(0, 0, distance));
        flight.init();
        assertFalse(movementSync.snapshotInput().jump());
        movementSync.setGliding(false);
        movementSync.onGround.set(true);
        flight.onTick();
        assertTrue(flight.isFinished(), "a handed-off flight must not restart takeoff after landing");
        assertLandingMessage(distance);
    }

    @Test
    void glideLostAfterHandoffWithoutLandingFailsEvenInsideArrivalRadius() {
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        ElytraFlyMovement flight = new ElytraFlyMovement(new Vector3d(movementSync.position.get()));
        flight.init();
        movementSync.setGliding(false);
        flight.onTick();
        assertTrue(flight.isFinished());
        assertTrue(messages.list.stream().anyMatch(message -> message.getFormattedMessage().equals(
                LangManager.get("movementsync.command.elytra.fly.failed"))));
    }

    private void assertLandingMessage(double distance) {
        boolean arrived = distance <= ElytraFlyMovement.DEFAULT_ARRIVAL_RADIUS;
        assertTrue(messages.list.stream().anyMatch(message -> message.getFormattedMessage().equals(
                LangManager.get(arrived ? "movementsync.command.elytra.fly.arrived"
                        : "movementsync.command.elytra.fly.failed"))));
        if (!arrived) {
            assertTrue(messages.list.stream().anyMatch(message -> message.getFormattedMessage()
                    .contains("remaining horizontal distance=" + distance)), "failure must report remaining distance");
        }
    }

    @Test
    void groundedGlideEndsAtOrOutsideArrivalRadius() {
        for (double distance : new double[]{10, 40}) {
            movementSync.setGliding(true);
            movementSync.onGround.set(true);
            ElytraFlyMovement flight = new ElytraFlyMovement(
                    new Vector3d(movementSync.position.get()).add(distance, 0, 0));
            flight.onTick();
            assertTrue(flight.isFinished());
        }
    }
}
