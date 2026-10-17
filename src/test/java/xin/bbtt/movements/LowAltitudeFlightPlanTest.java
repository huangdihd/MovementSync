package xin.bbtt.movements;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import java.util.Map;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.*;

final class LowAltitudeFlightPlanTest {
    @Test void distantGoalsAreAdmittedForRollingRoutes() {
        assertTrue(LowAltitudeFlightPlan.admissible(new Vector3d(8, 64, 8), new Vector3d(208, 64, 8)));
        assertTrue(LowAltitudeFlightPlan.admissible(new Vector3d(), new Vector3d(30000, 64, 8)));
        assertFalse(LowAltitudeFlightPlan.admissible(new Vector3d(), new Vector3d(Double.NaN, 64, 8)));
    }

    @Test void publicationAcceptsLivePerturbationAndRetainsRouteWhilePending() throws Exception {
        var sync = new xin.bbtt.MovementSync();
        var goal = new org.joml.Vector3i(200, 64, 8);
        long generation = sync.beginStaticNavigationRequest(goal);
        var destination = new Vector3d(200.5, 64, 8.5);
        var old = new WaypointFlightTarget(java.util.List.of(new Vector3d(20, 64, 8)), destination, false);
        var replacement = new WaypointFlightTarget(java.util.List.of(new Vector3d(40, 64, 8)), destination, false);
        var movement = new ElytraFlyMovement(destination, generation).withLowAltitudePlan(
                new LowAltitudeFlightPlan.Result(old, new Vector3d(), destination, 0, 0));
        var pending = new java.util.concurrent.CompletableFuture<LowAltitudeFlightPlan.Result>();
        var futureField = ElytraFlyMovement.class.getDeclaredField("lowPlan");
        futureField.setAccessible(true);
        futureField.set(movement, pending);
        var routeField = ElytraFlyMovement.class.getDeclaredField("lowRoute");
        routeField.setAccessible(true);
        var update = ElytraFlyMovement.class.getDeclaredMethod("updateLowRoute", xin.bbtt.MovementSync.class, PhysicsState.class);
        update.setAccessible(true);
        var live = new PhysicsState(new Vector3d(30, 64, 8), new Vector3d(1, .1, .4), false);
        update.invoke(movement, sync, live);
        assertSame(old, routeField.get(movement));
        pending.complete(new LowAltitudeFlightPlan.Result(replacement, new Vector3d(), destination, 0, 0));
        update.invoke(movement, sync, live);
        assertSame(replacement, routeField.get(movement), "geometric publication must not match old position or velocity");
        futureField.set(movement, java.util.concurrent.CompletableFuture.completedFuture(
                new LowAltitudeFlightPlan.Result(old, live.position(), new Vector3d(500, 64, 8), 0, 0)));
        update.invoke(movement, sync, live);
        assertSame(replacement, routeField.get(movement), "stale destination cannot replace the current corridor");
        sync.beginStaticNavigationRequest(new org.joml.Vector3i(500, 64, 8));
        futureField.set(movement, java.util.concurrent.CompletableFuture.completedFuture(
                new LowAltitudeFlightPlan.Result(old, live.position(), destination, 0, 0)));
        update.invoke(movement, sync, live);
        assertSame(replacement, routeField.get(movement), "cancelled generation cannot publish");
        movement.onStop();
    }

    @Test void cancellationInterruptsTheActualWorker() throws Exception {
        var entered = new java.util.concurrent.CountDownLatch(1);
        var interrupted = new java.util.concurrent.CountDownLatch(1);
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                entered.countDown();
                try { new java.util.concurrent.CountDownLatch(1).await(); }
                catch (InterruptedException expected) {
                    interrupted.countDown();
                    Thread.currentThread().interrupt();
                }
                return false;
            }
        };
        var state = new PhysicsState(new Vector3d(8, 64, 8), new Vector3d(0, 0, 1), false);
        var future = LowAltitudeFlightPlan.submit(world, state, new Vector3d(8, 63, 14),
                MovementParameters.DEFAULT, true);
        try {
            assertTrue(entered.await(5, TimeUnit.SECONDS));
            assertTrue(future.cancel(false));
            assertTrue(interrupted.await(5, TimeUnit.SECONDS));
        } finally { future.cancel(true); }
    }

    @Test void customDamageRejectsLocalControlsWithoutGlobalArbitration() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        };
        var state = new PhysicsState(new Vector3d(8, 64, 8), new Vector3d(0, 0, 1), false);
        var context = new xin.bbtt.flight.FlightContext(world, state, MovementParameters.DEFAULT,
                DamageParameters.NONE, 20, new Vector3d(8, 64, 14), 0, 0, 0, false, 2, false, Map.of());
        var config = new xin.bbtt.flight.FlightConfiguration(
                java.util.List.of(xin.bbtt.flight.VanillaGlideMode.INSTANCE),
                java.util.List.of((c, a, t) -> { fail("local restriction must not score global candidates"); return 0; }),
                java.util.List.of((c, a, before, after) -> new xin.bbtt.flight.DamageRule.Damage(1, 1)), false);
        var decision = new ElytraFlightController.Decision(0, 0, false, ElytraFlightController.Phase.CRUISE);
        assertFalse(ElytraFlyMovement.lowDecisionSafe(context, decision, config));
        var safe = new xin.bbtt.flight.FlightConfiguration(config.modes(), config.costs(), java.util.List.of(), false);
        assertTrue(ElytraFlyMovement.lowDecisionSafe(context, decision, safe));
    }

    @Test void jumpPreviewUsesWalkingPhysicsAndDescendingDeployment() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        };
        var ground = new PhysicsState(new Vector3d(8, 64, 8), new Vector3d(), true);
        var deployment = LowAltitudeFlightPlan.deployment(world, ground, MovementParameters.DEFAULT, 0, 0);
        assertNotNull(deployment);
        assertFalse(deployment.onGround());
        assertTrue(deployment.velocity().y <= 0);
        assertTrue(deployment.position().y > ground.position().y);
        assertEquals(ground.position().x, deployment.position().x);
    }

    @Test void unknownAndHazardousLaunchesFailClosed() {
        var ground = new PhysicsState(new Vector3d(8, 64, 8), new Vector3d(), true);
        World unknown = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
        };
        World fire = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return new BlockState("fire", 0, Map.of(), "empty", 0, false, "");
            }
        };
        assertNull(LowAltitudeFlightPlan.deployment(unknown, ground, MovementParameters.DEFAULT, 0, 0));
        assertNull(LowAltitudeFlightPlan.deployment(fire, ground, MovementParameters.DEFAULT, 0, 0));
    }

    @Test void sharedWorkerDoesNotReadWorldOnCallerThread() throws Exception {
        Thread caller = Thread.currentThread();
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                assertNotSame(caller, Thread.currentThread());
                assertEquals("low-altitude-flight-planner", Thread.currentThread().getName());
                return true;
            }
        };
        var state = new PhysicsState(new Vector3d(8, 64, 8), new Vector3d(0, 0, 1), false);
        Vector3d goal = new Vector3d(8, 63, 14);
        var result = LowAltitudeFlightPlan.submit(world, state, goal, MovementParameters.DEFAULT, true)
                .get(10, TimeUnit.SECONDS);
        assertNotNull(result);
        goal.zero();
        assertNotEquals(goal, result.destination());
        assertEquals(state.position(), result.origin());
    }
}
