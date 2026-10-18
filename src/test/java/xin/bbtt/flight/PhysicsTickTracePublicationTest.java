package xin.bbtt.flight;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;
import xin.bbtt.physics.ElytraFlightController;
import xin.bbtt.physics.FlightControlAdmission;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.tasks.updateMotionTask;
import xin.bbtt.world.World;

class PhysicsTickTracePublicationTest extends ElytraCommandTestSupport {
    // These manual publications exercise the diagnostic publication boundary, not physical simulation.
    private void publish(Vector3d position, Vector3d velocity) {
        movementSync.recordFlightTick(new MovementSync.FlightTickTrace(position, velocity,
                false, true, false, true, false, 31f, -12f, PlayerInput.NONE, 2.5));
    }

    @Test void disabledByDefaultAndDisablingClearsLatestUntilAnotherEnabledPublication() {
        assertNull(movementSync.getLastPhysicsTickTrace());
        publish(new Vector3d(1), new Vector3d(2));
        assertNull(movementSync.getLastPhysicsTickTrace());
        movementSync.setPhysicsTraceEnabled(true);
        assertNull(movementSync.getLastPhysicsTickTrace(), "enabling does not replay an old tick");
        publish(new Vector3d(3), new Vector3d(4));
        var first = movementSync.getLastPhysicsTickTrace();
        assertNotNull(first);
        movementSync.setPhysicsTraceEnabled(false);
        assertNull(movementSync.getLastPhysicsTickTrace());
        publish(new Vector3d(5), new Vector3d(6));
        assertNull(movementSync.getLastPhysicsTickTrace());
        movementSync.setPhysicsTraceEnabled(true);
        assertNull(movementSync.getLastPhysicsTickTrace());
        publish(new Vector3d(7), new Vector3d(8));
        assertEquals(first.sequence() + 1, movementSync.getLastPhysicsTickTrace().sequence());
        assertEquals(3, first.x());
    }

    @Test void publishedTraceCopiesMutableVectorsAndDoesNotAdvanceInputClock() {
        movementSync.setPhysicsTraceEnabled(true);
        var position = new Vector3d(1, 2, 3);
        var velocity = new Vector3d(4, 5, 6);
        long inputVersion = movementSync.physicsTickVersion();
        publish(position, velocity);
        var trace = movementSync.getLastPhysicsTickTrace();
        position.set(100, 200, 300);
        velocity.set(-100, -200, -300);
        assertEquals(new Vector3d(1, 2, 3), new Vector3d(trace.x(), trace.y(), trace.z()));
        assertEquals(new Vector3d(4, 5, 6), new Vector3d(trace.vx(), trace.vy(), trace.vz()));
        assertFalse(trace.onGround());
        assertTrue(trace.collidedX());
        assertFalse(trace.collidedZ());
        assertTrue(trace.gliding());
        assertFalse(trace.attached());
        assertEquals(31f, trace.yaw());
        assertEquals(-12f, trace.pitch());
        assertEquals(PlayerInput.NONE, trace.input());
        assertEquals(2.5, trace.fallDistance());
        assertEquals(inputVersion, trace.inputVersion());
        assertEquals(inputVersion, movementSync.physicsTickVersion());
    }

    @Test void delayedReaderRetainsOldSnapshotAcrossNewPublication() throws Exception {
        movementSync.setPhysicsTraceEnabled(true);
        var position = new Vector3d(1, 2, 3);
        var velocity = new Vector3d(4, 5, 6);
        publish(position, velocity);
        var first = movementSync.getLastPhysicsTickTrace();
        var held = new CountDownLatch(1);
        var published = new CountDownLatch(1);
        var executor = Executors.newSingleThreadExecutor();
        try {
            var reader = executor.submit(() -> {
                var retained = movementSync.getLastPhysicsTickTrace();
                held.countDown();
                assertTrue(published.await(5, TimeUnit.SECONDS));
                assertSame(first, retained);
                assertEquals(1, retained.x());
                assertEquals(6, retained.vz());
                assertEquals(first.ownership(), retained.ownership());
                return retained;
            });
            assertTrue(held.await(5, TimeUnit.SECONDS));
            position.set(10, 20, 30);
            velocity.set(40, 50, 60);
            publish(position, velocity);
            var latest = movementSync.getLastPhysicsTickTrace();
            assertNotSame(first, latest);
            assertEquals(first.sequence() + 1, latest.sequence());
            assertEquals(10, latest.x());
            published.countDown();
            assertSame(first, reader.get(5, TimeUnit.SECONDS));
        } finally {
            published.countDown();
            executor.shutdownNow();
            assertTrue(executor.awaitTermination(5, TimeUnit.SECONDS));
        }
    }

    private void airborneWorld() throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        });
        movementSync.position.set(new Vector3d(0, 20, 0));
        movementSync.velocity.set(new Vector3d(0, 0, .3));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
    }

    private PhysicsTickTrace admit(Object owner) throws Exception {
        airborneWorld();
        movementSync.setPhysicsTraceEnabled(true);
        var source = FlightControlAdmission.snapshot(movementSync);
        var decision = new ElytraFlightController.Decision(0, 0, false,
                ElytraFlightController.Phase.CRUISE);
        // Synthetic single-step lease isolates ownership diagnostics, not planner/terminal proof.
        movementSync.submitFlightControls(new FlightControlAdmission.Envelope(owner, source,
                List.of(new FlightControlAdmission.Step(decision, source.state())), null, () -> {}, List.of()));
        movementSync.snapshotInput();
        publish(movementSync.position.get(), movementSync.velocity.get());
        var trace = movementSync.getLastPhysicsTickTrace();
        assertTrue(trace.ownership().active());
        assertEquals(source.tick(), trace.ownership().sourceTick());
        assertEquals(source.generation(), trace.ownership().sourceGeneration());
        assertEquals(0, trace.ownership().cursor());
        assertFalse(trace.ownership().recovering());
        return trace;
    }

    @Test void ownerRevocationClearsActiveFlagWithoutChangingNavigationGeneration() throws Exception {
        var owner = new Object();
        var active = admit(owner);
        movementSync.revokeFlightControls(new Object());
        publish(movementSync.position.get(), movementSync.velocity.get());
        assertEquals(active.ownership(), movementSync.getLastPhysicsTickTrace().ownership());
        movementSync.revokeFlightControls(owner);
        publish(movementSync.position.get(), movementSync.velocity.get());
        var revoked = movementSync.getLastPhysicsTickTrace();
        assertEquals(active.navigationGeneration(), revoked.navigationGeneration());
        assertInactiveAfter(active, revoked);
    }

    @Test void replacementLeaseGetsDistinctActivationWithinSameNavigation() throws Exception {
        var oldOwner = new Object();
        var first = admit(oldOwner);
        var replacement = admit(new Object());
        assertEquals(first.navigationGeneration(), replacement.navigationGeneration());
        assertTrue(replacement.ownership().activation() > first.ownership().activation());
        movementSync.revokeFlightControls(oldOwner);
        publish(movementSync.position.get(), movementSync.velocity.get());
        assertEquals(replacement.ownership(), movementSync.getLastPhysicsTickTrace().ownership(),
                "late revocation of the previous owner must not describe the replacement as inactive");
    }

    @Test void navigationInvalidationChangesGenerationAsWellAsOwnership() throws Exception {
        var active = admit(new Object());
        movementSync.suspendNavigationForRecovery();
        publish(movementSync.position.get(), movementSync.velocity.get());
        var invalidated = movementSync.getLastPhysicsTickTrace();
        assertTrue(invalidated.navigationGeneration() > active.navigationGeneration());
        assertInactiveAfter(active, invalidated);
    }

    private static void assertInactiveAfter(PhysicsTickTrace active, PhysicsTickTrace inactive) {
        assertFalse(inactive.ownership().active());
        assertEquals(active.ownership().activation(), inactive.ownership().activation(),
                "revocation ends the lease; only a new admission allocates a new activation");
        assertEquals(-1, inactive.ownership().sourceTick());
        assertEquals(-1, inactive.ownership().sourceGeneration());
        assertEquals(-1, inactive.ownership().admittedAt());
        assertEquals(-1, inactive.ownership().cursor());
        assertTrue(active.ownership().active(), "previously published ownership remains detached");
    }

    @Test void actualMotionTaskPublishesPostPhysicsStateAndLeavesPriorTickDetached() throws Exception {
        var running = Bot.class.getDeclaredField("running");
        running.setAccessible(true);
        var previousRunning = running.get(Bot.INSTANCE);
        var previousServer = Bot.INSTANCE.getServer();
        try {
            movementSync.onRecoveryTeleport();
            airborneWorld();
            running.set(Bot.INSTANCE, true);
            Bot.INSTANCE.setServer(Server.Game);
            movementSync.setPhysicsTraceEnabled(true);
            var task = new updateMotionTask();
            task.run();
            var first = movementSync.getLastPhysicsTickTrace();
            assertNotNull(first, "must pass the actual task's position/running gates");
            assertTrue(first.gliding());
            assertTrue(first.y() < 20, "real gliding physics advances the starting state");
            assertEquals(movementSync.position.get(), new Vector3d(first.x(), first.y(), first.z()));
            assertEquals(movementSync.velocity.get(), new Vector3d(first.vx(), first.vy(), first.vz()));
            double firstY = first.y();
            task.run();
            var second = movementSync.getLastPhysicsTickTrace();
            assertEquals(first.sequence() + 1, second.sequence());
            assertTrue(second.inputVersion() > first.inputVersion());
            assertTrue(second.y() < firstY);
            assertEquals(firstY, first.y());
        } finally {
            running.set(Bot.INSTANCE, previousRunning);
            Bot.INSTANCE.setServer(previousServer);
        }
    }
}
