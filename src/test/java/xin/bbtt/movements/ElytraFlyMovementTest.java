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

    @ParameterizedTest
    @ValueSource(ints = {0, 1, 2})
    void waterCompletesNormalSafeAndIndependentLandingWithoutClearingFlag(int mode) throws Exception {
        movementSync.getMovementController().pause();
        var water = java.util.stream.IntStream.range(0, xin.bbtt.Block.BlockStateParser.getBlockStateRegistrySize())
                .mapToObj(xin.bbtt.Block.BlockStateParser.Instance::parseStateId)
                .filter(block -> block.blockName().equals("water") && "0".equals(block.getProperty("level")))
                .findFirst().orElseThrow();
        var world = new xin.bbtt.world.World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d p) { return water; }
        };
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        ElytraFlyMovement flight;
        if (mode == 2) {
            movementSync.cancelNavigation();
            flight = (ElytraFlyMovement) movementSync.getMovementController().getMovements().getFirst();
        } else {
            flight = new ElytraFlyMovement(new Vector3d(movementSync.position.get()));
            if (mode == 1) flight.forSafeLanding();
        }
        flight.init();
        flight.onTick();
        assertTrue(flight.isFinished());
        assertTrue(movementSync.isGliding());
        assertFalse(movementSync.onGround.get());
    }

    @Test
    void cancellationOwnsIndependentLandingAndRepeatedStopsPreserveIt() {
        movementSync.getMovementController().pause();
        movementSync.setGliding(true);
        movementSync.setActiveGoal(new org.joml.Vector3i(100, 0, 0));
        movementSync.cancelNavigation();
        var landing = assertInstanceOf(ElytraFlyMovement.class,
                movementSync.getMovementController().getMovements().getFirst());
        assertNull(movementSync.getActiveGoal());
        assertNull(movementSync.getGotoRecovery().getTask());
        movementSync.cancelNavigation();
        movementSync.cancelNavigation();
        assertSame(landing, movementSync.getMovementController().getMovements().getFirst());
        assertFalse(landing.isFinished());
        movementSync.beginGotoNavigationRequest(new org.joml.Vector3i(200, 0, 0), false, false);
        assertTrue(movementSync.getMovementController().getMovements().isEmpty());
    }

    @Test
    void groundAndDeathCancellationDoNotLand() {
        movementSync.getMovementController().pause();
        movementSync.cancelNavigation();
        assertTrue(movementSync.getMovementController().getMovements().isEmpty());
        movementSync.setGliding(true);
        new xin.bbtt.listeners.MovementParametersPacketListener().packetReceived(null,
                new org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundSetHealthPacket(0, 20, 5));
        assertTrue(movementSync.getMovementController().getMovements().isEmpty());
        assertFalse(movementSync.isGliding());
    }

    @Test
    void forcedLandingFinishesAsFailureWithoutPredictionFailure() throws Exception {
        xin.bbtt.world.World world = new xin.bbtt.world.World() {
            @Override public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }
            @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) { return minY < 0; }
            @Override public java.util.OptionalDouble findHighestCollisionTopExcluding(
                    double minX, double minY, double minZ, double maxX, double maxY, double maxZ,
                    java.util.Set<org.joml.Vector3i> exclusions) {
                return minY <= 0 && maxY >= 0 ? java.util.OptionalDouble.of(0) : java.util.OptionalDouble.empty();
            }
        };
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.position.set(new Vector3d(0, 1, 0));
        movementSync.velocity.set(new Vector3d(0, -0.2, 0.3));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        ElytraFlyMovement flight = new ElytraFlyMovement(new Vector3d(0, 0, 800));
        flight.init();
        for (int tick = 0; tick < 200 && !movementSync.onGround.get(); tick++) {
            flight.onTick();
            assertFalse(flight.isFinished(), "steering must continue until touchdown");
            var result = new xin.bbtt.physics.GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(), false)
                    .step(world, new xin.bbtt.physics.PhysicsState(movementSync.position.get(),
                            movementSync.velocity.get(), false), 0.299, 0.6);
            movementSync.position.set(result.position());
            movementSync.velocity.set(result.velocity());
            movementSync.onGround.set(result.onGround());
        }
        assertTrue(movementSync.onGround.get());
        flight.onTick();
        assertTrue(flight.isFinished());
        assertTrue(messages.list.stream().anyMatch(message -> message.getFormattedMessage().equals(
                LangManager.get("movementsync.command.elytra.fly.failed"))));
        assertFalse(messages.list.stream().anyMatch(message -> message.getFormattedMessage().contains("Flight prediction failed")));
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
