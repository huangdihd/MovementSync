package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Entity.Entity;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.flight.LocalGotoController;
import xin.bbtt.physics.EntityFlightTarget;
import xin.bbtt.physics.FlightControlAdmission;
import xin.bbtt.physics.LocalLookaheadSolver;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.physics.WaypointFlightTarget;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class AirborneCaptureRouteTest extends ElytraCommandTestSupport {
    @Test void movingBodyAndSmallEmaTailKeepRouteAndAllowThreeTickDelayedPublication() throws Exception {
        try (var fixture = new Fixture()) {
            var competingPlan = new CompletableFuture<LowAltitudeFlightPlan.Result>();
            set(fixture.movement, "lowPlan", competingPlan);
            fixture.update();
            var route = fixture.capture();
            assertTrue(competingPlan.isCancelled(), "capture owns planning, not a competing low-plan worker");
            assertNull(get(fixture.movement, "lowPlan"));
            assertTrue(fixture.request(route));
            assertTrue(fixture.world.entered.await(5, TimeUnit.SECONDS));
            for (int tick = 0; tick < 3; tick++) {
                movementSync.position.set(new Vector3d(0, 100, .3 * (tick + 1)));
                movementSync.completePhysicsStep();
                fixture.entity.getPosition().z += .005;
                fixture.target.sample(fixture.world);
                fixture.update();
                assertTrue(fixture.target.velocity().lengthSquared() > 0, "exercise the EMA, not an exact stop");
                var liveGoal = WaypointFlightTarget.pursuitAim(fixture.target,
                        FlightControlAdmission.snapshot(movementSync).state())
                        .fma((int) get(fixture.movement, "PURSUIT_PREDICTION_TICKS"), fixture.target.velocity());
                assertTrue(liveGoal.distance(route.goal()) <= PlayerPose.HALF_WIDTH);
                assertSame(route, fixture.capture(), "body movement and a sub-width endpoint tail are not new intent");
                assertNull(get(fixture.movement, "lowPlan"));
                assertFalse(fixture.request(fixture.capture()), "the blocked real worker remains single-flight");
            }
            var rebuilt = new WaypointFlightTarget(List.of(movementSync.position.get(), route.goal()), route.goal(), false);
            assertFalse(route.sameCorridor(rebuilt), "capture stability must not weaken global corridor equality");
            fixture.world.release.countDown();
            assertTrue(fixture.controller.awaitIdle(10, TimeUnit.SECONDS));
            assertNotNull(fixture.controller.latest(), "a three-tick-old worker must retain publication rights");
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, fixture.controller.latest().status());
            assertInstanceOf(FlightControlAdmission.Envelope.class, pendingEnvelope(),
                    "verify real proof publication, not only a retained private route reference");
        }
    }

    @Test void realEndpointDriftReplacesRouteAndCancelsBlockedPublication() throws Exception {
        try (var fixture = new Fixture()) {
            fixture.update();
            var original = fixture.capture();
            assertTrue(fixture.request(original));
            assertTrue(fixture.world.entered.await(5, TimeUnit.SECONDS));
            // Vertical drift changes the endpoint without triggering the horizontal motion exit.
            fixture.entity.getPosition().y += 1;
            fixture.target.sample(fixture.world);
            fixture.update();
            var replacement = fixture.capture();
            assertNotSame(original, replacement);
            assertTrue(original.goal().distance(replacement.goal()) > PlayerPose.HALF_WIDTH);
            assertFalse(original.sameCorridor(replacement));
            assertFalse(fixture.request(replacement), "new intent invalidates the old request without overlapping it");
            fixture.world.release.countDown();
            assertTrue(fixture.controller.awaitIdle(5, TimeUnit.SECONDS));
            assertNull(fixture.controller.latest());
            assertNull(pendingEnvelope(), "obsolete geometry must never publish controls");
        }
    }

    enum Exit { TARGET_MOVES, TARGET_STOPS_GLIDING, VISIBILITY_BLOCKED }

    @ParameterizedTest
    @EnumSource(Exit.class)
    void losingCaptureConditionsCancelsBlockedPublication(Exit exit) throws Exception {
        try (var fixture = new Fixture()) {
            fixture.update();
            assertTrue(fixture.request(fixture.capture()));
            assertTrue(fixture.world.entered.await(5, TimeUnit.SECONDS));
            switch (exit) {
                case TARGET_MOVES -> fixture.entity.getPosition().x += 1;
                case TARGET_STOPS_GLIDING -> fixture.entity.getMetadata().put("shared_flags", (byte) 0);
                case VISIBILITY_BLOCKED -> fixture.world.blocked = true;
            }
            fixture.target.sample(fixture.world);
            // Keep ordinary replanning outside this cancellation-focused test.
            set(fixture.movement, "lowPlan", new CompletableFuture<LowAltitudeFlightPlan.Result>());
            fixture.update();
            assertEquals(false, get(fixture.movement, "capturingAirborneTarget"));
            assertNull(fixture.capture());
            fixture.world.release.countDown();
            assertTrue(fixture.controller.awaitIdle(5, TimeUnit.SECONDS));
            assertNull(fixture.controller.latest());
            assertNull(pendingEnvelope());
        }
    }

    @Test void stableCaptureDoesNotExtendEightTickSourceFreshness() throws Exception {
        try (var fixture = new Fixture()) {
            fixture.update();
            var original = fixture.capture();
            assertTrue(fixture.request(original));
            assertTrue(fixture.world.entered.await(5, TimeUnit.SECONDS));
            for (int tick = 0; tick < 9; tick++) movementSync.completePhysicsStep();
            fixture.update();
            assertSame(original, fixture.capture());
            fixture.world.release.countDown();
            assertTrue(fixture.controller.awaitIdle(5, TimeUnit.SECONDS));
            assertNull(fixture.controller.latest(), "route retention must not license stale physics sources");
            assertNull(pendingEnvelope());
        }
    }

    private final class Fixture implements AutoCloseable {
        final GatedWorld world = new GatedWorld();
        final Entity entity;
        final EntityFlightTarget target;
        final ElytraFlyMovement movement;
        final LocalGotoController controller;

        Fixture() throws Exception {
            movementSync.getMovementController().pause();
            set(movementSync, "world", world);
            movementSync.position.set(new Vector3d(0, 100, 0));
            movementSync.velocity.set(new Vector3d(0, 0, .3));
            movementSync.onGround.set(false);
            movementSync.setGliding(true);
            entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                    0, 100, 40, 0, 0, 0, new Vector3d());
            entity.getMetadata().put("shared_flags", (byte) 0x80);
            world.getEntities().put(99, entity);
            target = new EntityFlightTarget(99, 0, new Vector3d(0, 100, 40));
            target.sample(world);
            var terminal = new Vector3d(0, 100, 2);
            var approach = new WaypointFlightTarget(List.of(movementSync.position.get(), terminal), terminal, false);
            movement = new ElytraFlyMovement(target, movementSync.getNavigationGeneration(), 2).forFollow()
                    .withLowAltitudePlan(new LowAltitudeFlightPlan.Result(approach,
                            movementSync.position.get(), target.position(), 0, 0));
            controller = (LocalGotoController) get(movement, "localGoto");
            movementSync.claimFlightControlOwner(movement, FlightControlAdmission.Mode.LOCAL_SOLVER);
        }

        void update() throws Exception {
            var method = ElytraFlyMovement.class.getDeclaredMethod("updateLowRoute", MovementSync.class, PhysicsState.class);
            method.setAccessible(true);
            method.invoke(movement, movementSync, new PhysicsState(movementSync.position.get(),
                    movementSync.velocity.get(), false, 0, PlayerPose.GLIDING));
        }

        WaypointFlightTarget capture() throws Exception {
            return (WaypointFlightTarget) get(movement, "airborneCaptureRoute");
        }

        boolean request(WaypointFlightTarget route) {
            assertNotNull(route, "the seeded terminal approach must enter capture instead of local climb");
            return controller.request(movementSync, movement, FlightControlAdmission.snapshot(movementSync),
                    route, route.goal(), false, () -> {},
                    new LocalLookaheadSolver.MovingTarget(target.position(), target.velocity()));
        }

        @Override public void close() throws Exception {
            world.release.countDown();
            controller.close();
            assertTrue(controller.awaitIdle(5, TimeUnit.SECONDS));
            var plan = (CompletableFuture<?>) get(movement, "lowPlan");
            if (plan != null) plan.cancel(false);
        }
    }

    private Object pendingEnvelope() throws Exception {
        return get(get(movementSync, "flightControls"), "pending");
    }

    private static Object get(Object owner, String name) throws Exception {
        return field(owner, name).get(owner);
    }

    private static void set(Object owner, String name, Object value) throws Exception {
        field(owner, name).set(owner, value);
    }

    private static Field field(Object owner, String name) throws Exception {
        var field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field;
    }

    private static final class GatedWorld extends World {
        private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        final CountDownLatch entered = new CountDownLatch(1);
        final CountDownLatch release = new CountDownLatch(1);
        private final AtomicBoolean gated = new AtomicBoolean();
        volatile boolean blocked;

        @Override public boolean chunkLoaded(int x, int z) {
            // Only gate the real solver, never visibility checks on the movement thread.
            if (Thread.currentThread().getName().equals("flight-local-goto") && gated.compareAndSet(false, true)) {
                entered.countDown();
                try {
                    assertTrue(release.await(10, TimeUnit.SECONDS), "test must release terrain worker");
                } catch (InterruptedException error) {
                    Thread.currentThread().interrupt();
                    throw new AssertionError(error);
                }
            }
            return true;
        }

        @Override public BlockState getBlockStateAt(Vector3d position) {
            return blocked && position.z >= 10 && position.z < 11 ? STONE : super.getBlockStateAt(position);
        }
    }
}
