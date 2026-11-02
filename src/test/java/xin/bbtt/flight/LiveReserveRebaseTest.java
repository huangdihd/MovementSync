package xin.bbtt.flight;

import java.lang.reflect.Field;
import java.util.List;
import java.util.OptionalInt;
import java.util.UUID;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Delayed pursuit proofs must retain their original authority and match the live body. */
class LiveReserveRebaseTest extends ElytraCommandTestSupport {
    private static final Vector3d GOAL = new Vector3d(0, 100, 40);
    enum Change { ROUTE, WORLD, EPOCH, GENERATION, ATTACHMENT, ADVANCE_TICK, EXPIRE_ORIGINAL, REVOKE_AND_RENEW, ROUTE_REVOKE_AND_RENEW }

    @ParameterizedTest @EnumSource(Change.class)
    void changeDuringLiveProofCannotPublishOrReviveOriginal(Change change) throws Exception {
        var world = new TwoStageWorld();
        Object owner = new Object();
        var source = install(world, owner);
        var route = route(source.state().position());
        try (var controller = new LocalGotoController()) {
            start(controller, owner, source, route);
            assertTrue(world.initialEntered.await(5, TimeUnit.SECONDS));
            moveToLiveBody(4);
            world.initialRelease.countDown();
            assertTrue(world.liveEntered.await(5, TimeUnit.SECONDS), "must gate the live rebuild, not initial solve");
            switch (change) {
                case ROUTE -> controller.observeRoute(movementSync, owner,
                        new WaypointFlightTarget(List.of(new Vector3d(0, 100, 0), new Vector3d(40, 100, 0)),
                                new Vector3d(40, 100, 0), false, false, 64));
                case WORLD -> field(MovementSync.class, "world").set(movementSync, new World());
                case EPOCH -> movementSync.invalidateFlightControls();
                case GENERATION -> ((AtomicLong) field(MovementSync.class, "navigationGeneration").get(movementSync)).incrementAndGet();
                case ATTACHMENT -> {
                    var rocket = new xin.bbtt.Entity.Entity(9001, new UUID(0, 9001),
                            org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType.FIREWORK_ROCKET,
                            0, 100, 0, 0, 0, 0, new Vector3d());
                    rocket.getMetadata().put("attached_to_target", OptionalInt.of(movementSync.entityId));
                    world.getEntities().put(9001, rocket);
                    assertTrue(movementSync.hasAttachedFirework());
                }
                case ADVANCE_TICK -> advance(1);
                case EXPIRE_ORIGINAL -> advance(5);
                case REVOKE_AND_RENEW, ROUTE_REVOKE_AND_RENEW -> {
                    if (change == Change.ROUTE_REVOKE_AND_RENEW) controller.invalidateRoute(movementSync, owner);
                    else movementSync.invalidateForwardFlightEmergency(owner);
                    assertTrue(movementSync.registerForwardFlightEmergency(owner,
                            FlightControlAdmission.snapshot(movementSync), 123f, route));
                }
            }
            world.liveRelease.countDown();
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertNull(controller.latest(), "cancelled live proof must not publish a result: " + change);
            assertNull(admissionField("pending"), "cancelled live proof must not publish a lease: " + change);
        } finally {
            world.initialRelease.countDown();
            world.liveRelease.countDown();
        }
    }

    @ParameterizedTest @ValueSource(ints = {4, 8})
    void delayedOrdinaryRebaseStillPublishesInsideOriginalAgeLimit(int delay) throws Exception {
        var world = new TwoStageWorld();
        Object owner = new Object();
        var source = install(world, owner);
        try (var controller = new LocalGotoController()) {
            start(controller, owner, source, route(source.state().position()));
            assertTrue(world.initialEntered.await(5, TimeUnit.SECONDS));
            moveToLiveBody(delay);
            world.initialRelease.countDown();
            assertTrue(world.liveEntered.await(5, TimeUnit.SECONDS));
            world.liveRelease.countDown();
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            var envelope = assertInstanceOf(FlightControlAdmission.Envelope.class, admissionField("pending"));
            assertEquals(delay, envelope.source().tick());
            assertEquals(FlightControlAdmission.snapshot(movementSync), envelope.source());
            assertEquals(envelope.source().state(), envelope.steps().get(0).expected());
            assertTrue(envelope.requireAirborne());
            assertEquals(6, envelope.recovery().size());
            assertEquals(1, envelope.handoff().size());
            movementSync.snapshotInput();
            assertTrue(movementSync.flightControlOwnership().active());
        } finally {
            world.initialRelease.countDown();
            world.liveRelease.countDown();
        }
    }

    @Test void delayedRebaseMustNotOverwriteNewerEmergencyHeading() throws Exception {
        var world = new TwoStageWorld();
        Object owner = new Object();
        var source = install(world, owner);
        var route = route(source.state().position());
        try (var controller = new LocalGotoController()) {
            start(controller, owner, source, route);
            assertTrue(world.initialEntered.await(5, TimeUnit.SECONDS));
            moveToLiveBody(4);
            world.initialRelease.countDown();
            assertTrue(world.liveEntered.await(5, TimeUnit.SECONDS));
            var newer = FlightControlAdmission.snapshot(movementSync);
            assertTrue(movementSync.registerForwardFlightEmergency(owner, newer, 123f, route));
            world.liveRelease.countDown();
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            var envelope = assertInstanceOf(FlightControlAdmission.Envelope.class, admissionField("pending"));
            assertEquals(4, envelope.source().tick());
            assertEquals(123f, admissionField("emergencyYaw"),
                    "old source-time steering must not overwrite the newer observation's heading");
        } finally {
            world.initialRelease.countDown();
            world.liveRelease.countDown();
        }
    }

    @Test void sameTickVelocityPacketMustInvalidateTheLiveBodyProof() throws Exception {
        var world = new TwoStageWorld();
        Object owner = new Object();
        var source = install(world, owner);
        try (var controller = new LocalGotoController()) {
            var constructor = WaypointFlightTarget.class.getDeclaredConstructor(List.class,
                    Vector3d.class, boolean.class, boolean.class, double.class, World.class);
            constructor.setAccessible(true);
            var plannerRoute = constructor.newInstance(List.of(source.state().position(), GOAL),
                    GOAL, false, false, 64d, world);
            start(controller, owner, source, plannerRoute);
            assertTrue(world.initialEntered.await(5, TimeUnit.SECONDS));
            moveToLiveBody(4);
            world.initialRelease.countDown();
            assertTrue(world.liveEntered.await(5, TimeUnit.SECONDS));
            synchronized (movementSync) {
                // Same write/monitor semantics as VelocityPacketListener: no clock/epoch advance.
                movementSync.velocity.set(new Vector3d(0, 0, .35));
            }
            world.liveRelease.countDown();
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertEquals(4, movementSync.physicsTickVersion(), "velocity packets do not advance the clock");
            assertNull(controller.latest(), "same-tick body changes invalidate the entire live proof");
            assertNull(admissionField("pending"), "a stale full-episode proof must not reach admission");
        } finally {
            world.initialRelease.countDown();
            world.liveRelease.countDown();
        }
    }

    @Test void elapsedResourcesComposeWithoutSpendingAnOrdinaryRocket() {
        var resources = new HazardRecovery.Context(new int[] {1, 2}, 10, false, null);
        var entry = new LocalGotoController.DangerousEntry(
                new PhysicsState(new Vector3d(), new Vector3d(), false, 0, PlayerPose.GLIDING), 3);
        var afterRebaseAndEntry = LocalGotoController.resourcesAfterDangerousEntry(resources.afterTicks(4, false), entry, false);
        assertArrayEquals(new int[] {1, 2}, afterRebaseAndEntry.rocketFlightDurations());
        assertEquals(3, afterRebaseAndEntry.cooldownTicks());
        assertFalse(afterRebaseAndEntry.attached());
        assertEquals(0, afterRebaseAndEntry.guaranteedBoostTicks());
    }

    private void start(LocalGotoController controller, Object owner, FlightControlAdmission.Source source,
            WaypointFlightTarget route) {
        assertTrue(controller.request(movementSync, owner, source, route, GOAL, false,
                () -> fail("ordinary reserve must not execute a rocket callback"),
                new LocalLookaheadSolver.MovingTarget(GOAL, new Vector3d(0, 0, .8)),
                HazardRecovery.Context.EMPTY, false, false));
    }

    private FlightControlAdmission.Source install(World world, Object owner) throws Exception {
        field(MovementSync.class, "world").set(movementSync, world);
        movementSync.position.set(new Vector3d(0, 100, 0));
        movementSync.velocity.set(new Vector3d(0, 0, .3));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        return FlightControlAdmission.snapshot(movementSync);
    }
    private static WaypointFlightTarget route(Vector3d initial) {
        return new WaypointFlightTarget(List.of(initial, GOAL), GOAL, false, false, 64);
    }
    private void moveToLiveBody(int ticks) {
        synchronized (movementSync) {
            advance(ticks);
            movementSync.position.set(new Vector3d(20, 100, 0));
        }
    }

    private void advance(int ticks) {
        for (int tick = 0; tick < ticks; tick++) movementSync.completePhysicsStep();
    }
    private Object admissionField(String name) throws Exception {
        return field(FlightControlAdmission.class, name).get(field(MovementSync.class, "flightControls").get(movementSync));
    }
    private static Field field(Class<?> type, String name) throws Exception {
        var result = type.getDeclaredField(name);
        result.setAccessible(true);
        return result;
    }
    private static final class TwoStageWorld extends World {
        final CountDownLatch initialEntered = new CountDownLatch(1), initialRelease = new CountDownLatch(1);
        final CountDownLatch liveEntered = new CountDownLatch(1), liveRelease = new CountDownLatch(1);
        final AtomicBoolean initialGated = new AtomicBoolean(), liveGated = new AtomicBoolean();
        @Override public boolean chunkLoaded(int x, int z) {
            if (!Thread.currentThread().getName().equals("flight-local-goto")) return true;
            if (initialGated.compareAndSet(false, true)) gate(initialEntered, initialRelease);
            return true;
        }

        @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d position) {
            // The source starts at X=0 with Vz=.3; its twelve-tick solve cannot reach X=19.
            // Only the rebuilt proof starts at X=20. Geometry, not source line numbers,
            // distinguishes the live-body proof from all source-time terrain work.
            if (Thread.currentThread().getName().equals("flight-local-goto")
                    && position.x >= 19 && liveGated.compareAndSet(false, true))
                gate(liveEntered, liveRelease);
            return super.getBlockStateAt(position);
        }

        private static void gate(CountDownLatch entered, CountDownLatch release) {
            entered.countDown();
            try { assertTrue(release.await(5, TimeUnit.SECONDS), "test must release terrain worker"); }
            catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
                throw new AssertionError(exception);
            }
        }
    }
}
