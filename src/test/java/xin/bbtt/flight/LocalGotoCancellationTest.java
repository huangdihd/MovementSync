package xin.bbtt.flight;

import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class LocalGotoCancellationTest extends ElytraCommandTestSupport {
    private static final Vector3d GOAL = new Vector3d(0, 100, 40);

    @Test void obsoleteRouteStopsTerrainWorkAndReleasesSingleFlightWithoutPublication() throws Exception {
        var world = new GatedWorld(null, false);
        Object owner = new Object();
        var source = source(world, owner);
        var accepted = new AtomicInteger();
        var replacementGoal = new Vector3d(40, 100, 0);
        var replacement = route(source.state().position(), replacementGoal);
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source,
                    route(source.state().position(), GOAL), GOAL, true, accepted::incrementAndGet));
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            try {
                assertFalse(controller.request(movementSync, owner, source, replacement, replacementGoal),
                        "a replacement invalidates old geometry but must not queue another worker");
                assertEquals(1, world.queries.get(), "there must be only one terrain worker");
            } finally {
                world.release.countDown();
            }
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertDiscarded(controller, world, accepted, 500);
            assertTrue(controller.request(movementSync, owner, source, replacement, replacementGoal),
                    "cancellation must release busy, not poison the same-tick replacement");
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, controller.latest().status());
            var envelope = assertInstanceOf(FlightControlAdmission.Envelope.class, pendingEnvelope());
            assertTrue(envelope.corridor().sameCorridor(replacement));
            movementSync.snapshotInput();
            assertEquals(0, accepted.get(), "the cancelled worker's callback must never be admitted");
        } finally {
            world.release.countDown();
        }
    }

    @Test void equalGeometryRenewalDoesNotCancelTheBlockedRequest() throws Exception {
        var world = new GatedWorld(null, false);
        Object owner = new Object();
        var source = source(world, owner);
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source,
                    route(source.state().position(), GOAL), GOAL));
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            try {
                var equalRoute = route(source.state().position(), new Vector3d(GOAL));
                controller.observeRoute(movementSync, owner, equalRoute);
                for (int tick = 0; tick < 8; tick++) movementSync.completePhysicsStep();
                assertFalse(controller.request(movementSync, owner, source, equalRoute, GOAL));
            } finally {
                world.release.countDown();
            }
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, controller.latest().status());
            assertNotNull(pendingEnvelope());
        } finally {
            world.release.countDown();
        }
    }

    @Test void delayedPursuitRebuildsOrdinaryReserveFromTheLiveBody() throws Exception {
        var world = new GatedWorld(null, false);
        Object owner = new Object();
        var source = source(world, owner);
        movementSync.setGliding(true);
        var route = route(source.state().position(), GOAL);
        var tracking = new LocalLookaheadSolver.MovingTarget(GOAL, new Vector3d(0, 0, .8));
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, GOAL,
                    false, () -> fail("ordinary rebase must not dispatch a rocket"), tracking,
                    HazardRecovery.Context.EMPTY, false, false));
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            try {
                var state = source.state();
                var openWorld = new World() {
                    @Override public boolean chunkLoaded(int x, int z) { return true; }
                };
                for (int tick = 0; tick < 4; tick++) {
                    var next = new GlidingPhysics(0, -20, false, PlayerInput.NONE,
                            movementSync.getMovementParameters()).step(openWorld, state,
                                    PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                    state = new PhysicsState(next.position(), next.velocity(), false,
                            next.fallDistance(), PlayerPose.GLIDING);
                    movementSync.position.set(state.position());
                    movementSync.velocity.set(state.velocity());
                    movementSync.fallDistance.set(state.fallDistance());
                    movementSync.completePhysicsStep();
                }
            } finally {
                world.release.countDown();
            }
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            var envelope = assertInstanceOf(FlightControlAdmission.Envelope.class, pendingEnvelope());
            assertEquals(4, envelope.source().tick());
            assertEquals(FlightControlAdmission.snapshot(movementSync).state(), envelope.steps().get(0).expected());
            assertEquals(6, envelope.recovery().size());
            assertEquals(1, envelope.handoff().size());
            assertTrue(envelope.requireAirborne());
            movementSync.snapshotInput();
            assertTrue(movementSync.flightControlOwnership().active());
        } finally {
            world.release.countDown();
        }
    }

    @Test void sourceOlderThanEightTicksCancelsBeforeFurtherSearch() throws Exception {
        var world = new GatedWorld(null, false);
        Object owner = new Object();
        var source = source(world, owner);
        var route = route(source.state().position(), GOAL);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, GOAL,
                    true, accepted::incrementAndGet));
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            try {
                for (int tick = 0; tick < 9; tick++) movementSync.completePhysicsStep();
            } finally {
                world.release.countDown();
            }
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertDiscarded(controller, world, accepted, 500);
            var current = FlightControlAdmission.snapshot(movementSync);
            assertTrue(controller.request(movementSync, owner, current, route, GOAL));
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertEquals(current.tick(), controller.latest().tick());
            assertNotNull(pendingEnvelope());
        } finally {
            world.release.countDown();
        }
    }

    @Test void closeCancelsBlockedTerrainWorkAndRejectsFurtherRequests() throws Exception {
        var world = new GatedWorld(null, false);
        Object owner = new Object();
        var source = source(world, owner);
        var route = route(source.state().position(), GOAL);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, GOAL,
                    true, accepted::incrementAndGet));
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            try {
                controller.close();
                assertFalse(controller.request(movementSync, owner, source, route, GOAL),
                        "close must reject requests while the old worker is still blocked");
                assertEquals(1, world.queries.get());
            } finally {
                world.release.countDown();
            }
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertDiscarded(controller, world, accepted, 500);
            movementSync.completePhysicsStep();
            var current = FlightControlAdmission.snapshot(movementSync);
            assertFalse(controller.request(movementSync, owner, current, route, GOAL),
                    "finishing the cancelled worker must not reopen the controller");
            movementSync.snapshotInput();
            assertEquals(0, accepted.get());
        } finally {
            world.release.countDown();
        }
    }

    @Test void worldReplacementCancelsBlockedWorkWithoutAnEpochOrGenerationChange() throws Exception {
        var world = new GatedWorld(null, false);
        var replacementWorld = new GatedWorld(null, false);
        replacementWorld.release.countDown();
        Object owner = new Object();
        var source = source(world, owner);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source,
                    route(source.state().position(), GOAL), GOAL, true, accepted::incrementAndGet));
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            try {
                var current = source(replacementWorld, owner);
                assertSame(replacementWorld, movementSync.getWorld());
                assertEquals(source.tick(), current.tick());
                assertEquals(source.epoch(), current.epoch());
                assertEquals(source.generation(), current.generation());
            } finally {
                world.release.countDown();
            }
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertDiscarded(controller, world, accepted, 500);
            assertEquals(0, replacementWorld.queries.get(),
                    "an obsolete worker must not switch terrain underneath its captured source");
            movementSync.snapshotInput();
            assertEquals(0, accepted.get());
        } finally {
            world.release.countDown();
        }
    }

    @Test void epochInvalidationCancelsBlockedWorkAndAllowsSameTickReclaimedOwner() throws Exception {
        var world = new GatedWorld(null, false);
        Object owner = new Object();
        var source = source(world, owner);
        var route = route(source.state().position(), GOAL);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, GOAL,
                    true, accepted::incrementAndGet));
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            FlightControlAdmission.Source current;
            try {
                movementSync.invalidateFlightControls();
                movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
                current = FlightControlAdmission.snapshot(movementSync);
                assertNotEquals(source.epoch(), current.epoch());
                assertEquals(source.tick(), current.tick());
                assertEquals(source.generation(), current.generation());
                assertFalse(controller.request(movementSync, owner, current, route, GOAL),
                        "a new epoch must not start a worker before the obsolete one exits");
                assertEquals(1, world.queries.get());
            } finally {
                world.release.countDown();
            }
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertDiscarded(controller, world, accepted, 500);
            assertTrue(controller.request(movementSync, owner, current, route, GOAL),
                    "reclaiming the same owner must permit a same-tick request after cancellation");
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, controller.latest().status());
            assertEquals(current.epoch(), controller.latest().epoch());
            var envelope = assertInstanceOf(FlightControlAdmission.Envelope.class, pendingEnvelope());
            assertEquals(current.epoch(), envelope.source().epoch());
            movementSync.snapshotInput();
            assertEquals(0, accepted.get(), "the old epoch's callback must not be admitted");
        } finally {
            world.release.countDown();
        }
    }

    @Test void invalidationDuringSolverAbortsBeforeFallbackPreparation() throws Exception {
        var world = new GatedWorld("solvePrefix", false);
        Object owner = new Object();
        var source = source(world, owner);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source,
                    route(source.state().position(), GOAL), GOAL, true, accepted::incrementAndGet));
            assertTrue(world.entered.await(5, TimeUnit.SECONDS));
            try {
                controller.invalidateRoute(movementSync, owner);
            } finally {
                world.release.countDown();
            }
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertDiscarded(controller, world, accepted, 50_000);
            assertFalse(world.preparedEpisode.get(), "cancellation is not an empty solver result");
        } finally {
            world.release.countDown();
        }
    }

    @Test void invalidationInsideRocketBeamStopsTheWholeEpisode() throws Exception {
        assertRocketCancellation("advanceRocketSearchBranches");
    }

    @Test void invalidationInsideLifetimeProofStopsTheWholeEpisode() throws Exception {
        assertRocketCancellation("advanceRocketBranches");
    }

    private void assertRocketCancellation(String blockedMethod) throws Exception {
        var world = new GatedWorld(blockedMethod, true);
        Object owner = new Object();
        source(world, owner);
        movementSync.position.set(new Vector3d(16.5, 65.56, 282.8));
        movementSync.velocity.set(new Vector3d(0, .007474, .7228));
        movementSync.fallDistance.set(1.0);
        movementSync.pitch.set(-20f);
        movementSync.setGliding(true);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        var source = FlightControlAdmission.snapshot(movementSync);
        var goal = new Vector3d(16.5, 64, 608.5);
        var route = LowAltitudePlanner.plan(world, source.state(), goal, MovementParameters.DEFAULT);
        assertNotNull(route);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, goal, true,
                    accepted::incrementAndGet, null,
                    new HazardRecovery.Context(new int[] {1}, 0, false, null), true));
            assertTrue(world.entered.await(5, TimeUnit.SECONDS), "worker must enter " + blockedMethod);
            try {
                controller.invalidateRoute(movementSync, owner);
                assertFalse(controller.request(movementSync, owner, source, route, goal),
                        "invalidation cannot start an overlapping worker");
            } finally {
                world.release.countDown();
            }
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            assertDiscarded(controller, world, accepted, 500);
            movementSync.snapshotInput();
            assertEquals(0, accepted.get());
        } finally {
            world.release.countDown();
        }
    }

    private void assertDiscarded(LocalGotoController controller, GatedWorld world,
            AtomicInteger accepted, int maximumRemainingQueries) throws Exception {
        assertNull(controller.latest(), "obsolete computation must not publish even an UNPROVEN result");
        assertNull(pendingEnvelope(), "obsolete computation must not publish an admission envelope");
        assertEquals(0, accepted.get());
        int remaining = world.queries.get() - world.blockedAt;
        assertTrue(remaining < maximumRemainingQueries,
                "cooperative cancellation must bound terrain work after invalidation: " + remaining);
    }

    private FlightControlAdmission.Source source(World world, Object owner) throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.position.set(new Vector3d(0, 100, 0));
        movementSync.velocity.set(new Vector3d(0, 0, .3));
        movementSync.onGround.set(false);
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        return FlightControlAdmission.snapshot(movementSync);
    }

    private static WaypointFlightTarget route(Vector3d initial, Vector3d goal) {
        return new WaypointFlightTarget(List.of(initial, goal), goal, false, false, 64);
    }

    private Object pendingEnvelope() throws Exception {
        var controls = MovementSync.class.getDeclaredField("flightControls");
        controls.setAccessible(true);
        var pending = FlightControlAdmission.class.getDeclaredField("pending");
        pending.setAccessible(true);
        return pending.get(controls.get(movementSync));
    }

    /** Stack selection gates a real phase without sleeps or exposing production test hooks. */
    private static final class GatedWorld extends World {
        private static final BlockState STONE =
                new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        private final String blockedMethod;
        private final boolean roofed;
        private final AtomicBoolean gated = new AtomicBoolean();
        private final AtomicBoolean preparedEpisode = new AtomicBoolean();
        private final AtomicInteger queries = new AtomicInteger();
        private final CountDownLatch entered = new CountDownLatch(1);
        private final CountDownLatch release = new CountDownLatch(1);
        private volatile int blockedAt;

        private GatedWorld(String blockedMethod, boolean roofed) {
            this.blockedMethod = blockedMethod;
            this.roofed = roofed;
        }

        @Override public boolean chunkLoaded(int x, int z) {
            int count = queries.incrementAndGet();
            if (!gated.get() || "solvePrefix".equals(blockedMethod)) {
                var stack = Thread.currentThread().getStackTrace();
                boolean matches = blockedMethod == null;
                for (var frame : stack) {
                    if (frame.getMethodName().equals("prepareEpisode")) preparedEpisode.set(true);
                    if (frame.getMethodName().equals(blockedMethod)) matches = true;
                }
                if (matches && gated.compareAndSet(false, true)) {
                    blockedAt = count;
                    entered.countDown();
                    try {
                        assertTrue(release.await(5, TimeUnit.SECONDS), "test must release terrain query");
                    } catch (InterruptedException error) {
                        Thread.currentThread().interrupt();
                        throw new AssertionError(error);
                    }
                }
            }
            return true;
        }

        @Override public BlockState getBlockStateAt(Vector3d position) {
            return roofed && (position.y < 64 || position.y >= 80)
                    ? STONE : super.getBlockStateAt(position);
        }
    }
}
