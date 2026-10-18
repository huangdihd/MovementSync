package xin.bbtt.flight;

import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class LocalGotoControllerTest extends ElytraCommandTestSupport {
    private static final Vector3d GOAL = new Vector3d(0, 100, 40);
    private static World air() {
        return new World() { @Override public boolean chunkLoaded(int x, int z) { return true; } };
    }
    private WaypointFlightTarget route() {
        return new WaypointFlightTarget(List.of(new Vector3d(0, 100, 0), GOAL), GOAL, true);
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

    @Test void workerIsOffThreadDropsBacklogAndRejectsEpochStalePublication() throws Exception {
        var entered = new CountDownLatch(1);
        var release = new CountDownLatch(1);
        var queries = new AtomicInteger();
        Thread caller = Thread.currentThread();
        World gated = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                assertNotSame(caller, Thread.currentThread());
                queries.incrementAndGet();
                entered.countDown();
                try { assertTrue(release.await(5, TimeUnit.SECONDS)); }
                catch (InterruptedException error) { throw new AssertionError(error); }
                return true;
            }
        };
        Object owner = new Object();
        var source = source(gated, owner);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route(), GOAL, true, accepted::incrementAndGet));
            assertTrue(entered.await(5, TimeUnit.SECONDS));
            try {
                assertFalse(controller.request(movementSync, owner, source, route(), GOAL));
                movementSync.invalidateFlightControls();
            } finally { release.countDown(); }
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertNull(controller.latest());
            assertEquals(0, accepted.get());
            assertTrue(queries.get() > 0);
        }
    }

    @Test void unknownTerrainIsExplicitUnprovenNotLandingOrRocket() throws Exception {
        Object owner = new Object();
        var source = source(new World() {}, owner);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route(), GOAL, true, accepted::incrementAndGet));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertEquals(LocalGotoController.Status.UNPROVEN, controller.latest().status());
            assertEquals(0, controller.latest().prefixTicks());
            assertEquals(0, controller.latest().reserveTicks());
            assertFalse(controller.latest().rocket());
            assertEquals(0, accepted.get());
        }
    }

    @Test void openAirProducesFinitePrefixWithoutExecutingCallbackOnWorker() throws Exception {
        Object owner = new Object();
        var source = source(air(), owner);
        var accepted = new AtomicInteger();
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route(), GOAL, accepted::incrementAndGet));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            var result = controller.latest();
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, result.status());
            assertTrue(result.prefixTicks() > 0 && result.prefixTicks() <= 12);
            assertTrue(result.reserveTicks() >= 0 && result.reserveTicks() <= 6);
            assertFalse(result.rocket());
            assertEquals(0, accepted.get());
            assertEquals(source.tick(), result.tick());
            assertEquals(source.epoch(), result.epoch());
            assertEquals(source.generation(), result.generation());
            assertFalse(controller.request(movementSync, owner, source, route(), GOAL));
        }
    }

    @Test void tickZeroDeduplicatesStateButAllowsLegacyDriverToAdvance() throws Exception {
        Object owner = new Object();
        var initial = source(air(), owner);
        assertEquals(0, initial.tick());
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, initial, route(), GOAL));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertFalse(controller.request(movementSync, owner, initial, route(), GOAL));
            movementSync.position.set(new Vector3d(0, 100, 1));
            var advanced = FlightControlAdmission.snapshot(movementSync);
            assertEquals(initial.tick(), advanced.tick());
            assertTrue(controller.request(movementSync, owner, advanced, route(), GOAL));
            controller.awaitIdle(5, TimeUnit.SECONDS);
        }
    }

    @Test void arrivalDoesNotRequestRocketAtCorrectGoalElevation() throws Exception {
        Object owner = new Object();
        var initial = source(air(), owner);
        Vector3d nearby = new Vector3d(0, 100, 10);
        var target = new WaypointFlightTarget(List.of(initial.state().position(), nearby), nearby, true);
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, initial, target, nearby, true, () -> {}));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertFalse(controller.latest().rocket());
            assertNull(controller.latest().powered(), "arrival must not evaluate unnecessary thrust");
        }
    }

    private static World floor(boolean hot) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) { return minY < 10; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 10 ? new BlockState(hot ? "magma_block" : "stone", 1,
                        Map.of(), "block", 1.5, true, "STONE") : super.getBlockStateAt(position);
            }
        };
    }
    @Test void landingRequiresTargetHeightHorizontalRadiusActualSupportAndNoHazard() {
        var state = new PhysicsState(new Vector3d(0, 10, 0), new Vector3d(), true);
        assertTrue(LocalGotoController.normalLanding(floor(false), state, new Vector3d(0, 10, 0)));
        assertTrue(LocalGotoController.normalLanding(floor(false), state, new Vector3d(0, 11, 0)));
        assertFalse(LocalGotoController.normalLanding(floor(false), state, new Vector3d(0, 11.01, 0)));
        assertFalse(LocalGotoController.normalLanding(floor(false), state, new Vector3d(2.01, 10, 0)));
        assertFalse(LocalGotoController.normalLanding(air(), state, new Vector3d(0, 10, 0)));
        assertFalse(LocalGotoController.normalLanding(floor(true), state, new Vector3d(0, 10, 0)));
        assertFalse(LocalGotoController.normalLanding(new World() {}, state, new Vector3d(0, 10, 0)));
    }

    @Test void brakingReplayIsFiniteAndCannotTreatWrongHeightGroundAsSuccess() {
        var control = new ElytraFlightController.Decision(0, -45, false, ElytraFlightController.Phase.LANDING);
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .3), false);
        var replay = LocalGotoController.replay(air(), MovementParameters.DEFAULT, initial,
                control, false, 6, GOAL, true);
        assertEquals(6, replay.steps().size());
        assertTrue(replay.steps().stream().noneMatch(step -> step.control().firework()));
        var falling = new PhysicsState(new Vector3d(0, 10.02, 0), new Vector3d(0, -.2, 0), false);
        assertTrue(LocalGotoController.replay(floor(false), MovementParameters.DEFAULT, falling,
                control, false, 6, new Vector3d(0, 100, 0), true).steps().isEmpty());
    }
}
