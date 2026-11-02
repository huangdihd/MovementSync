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

    @Test void stalePursuitRouteIsRevokedUntilFreshForwardReplacementPublishes() throws Exception {
        var sync = new xin.bbtt.MovementSync();
        sync.position.set(new Vector3d(45, 70, 0));
        sync.velocity.set(new Vector3d(1, 0, 0));
        sync.onGround.set(false);
        sync.setGliding(true);
        var state = new PhysicsState(sync.position.get(), sync.velocity.get(), false);
        var target = new EntityFlightTarget(99, 6, new Vector3d(80, 70, 0));
        var oldDestination = new Vector3d(40, 70, 0);
        var oldRoute = new WaypointFlightTarget(java.util.List.of(
                new Vector3d(0, 70, 0), oldDestination), oldDestination, false, false, 8);
        var movement = new ElytraFlyMovement(target, sync.getNavigationGeneration(),
                ElytraFlyMovement.DEFAULT_ARRIVAL_RADIUS).withLowAltitudePlan(
                        new LowAltitudeFlightPlan.Result(oldRoute, new Vector3d(), oldDestination, 0, 0));
        var pending = new java.util.concurrent.CompletableFuture<LowAltitudeFlightPlan.Result>();
        var futureField = ElytraFlyMovement.class.getDeclaredField("lowPlan");
        futureField.setAccessible(true);
        futureField.set(movement, pending);
        var routeField = ElytraFlyMovement.class.getDeclaredField("lowRoute");
        routeField.setAccessible(true);
        var staleField = ElytraFlyMovement.class.getDeclaredField("lowPursuitRouteStale");
        staleField.setAccessible(true);
        var update = ElytraFlyMovement.class.getDeclaredMethod("updateLowRoute",
                xin.bbtt.MovementSync.class, PhysicsState.class);
        update.setAccessible(true);

        sync.claimFlightControlOwner(movement, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var source = FlightControlAdmission.snapshot(sync);
        assertTrue(sync.registerForwardFlightEmergency(movement, source, -90, oldRoute));
        update.invoke(movement, sync, state);

        assertSame(oldRoute, routeField.get(movement),
                "replacement planning may retain geometry for bookkeeping, not steering ownership");
        assertTrue(staleField.getBoolean(movement));
        assertNull(flightControlField(sync, "emergencySource"),
                "the passed route must immediately lose fallback steering rights");

        sync.setGliding(false); // Keep this publication test from launching another planner worker.
        var replacementDestination = new Vector3d(80, 70, 0);
        var replacement = new WaypointFlightTarget(java.util.List.of(
                state.position(), replacementDestination), replacementDestination, false, false, 8);
        pending.complete(new LowAltitudeFlightPlan.Result(
                replacement, state.position(), replacementDestination, 0, 0));
        update.invoke(movement, sync, state);

        assertSame(replacement, routeField.get(movement));
        assertFalse(staleField.getBoolean(movement));

        futureField.set(movement, java.util.concurrent.CompletableFuture.completedFuture(
                new LowAltitudeFlightPlan.Result(oldRoute, state.position(), replacementDestination, 0, 0)));
        update.invoke(movement, sync, state);
        assertSame(replacement, routeField.get(movement),
                "a result whose terminal tangent is already behind the body cannot publish");
        movement.onStop();
    }

    private static Object flightControlField(xin.bbtt.MovementSync sync, String name) throws Exception {
        var controls = xin.bbtt.MovementSync.class.getDeclaredField("flightControls");
        controls.setAccessible(true);
        var field = FlightControlAdmission.class.getDeclaredField(name);
        field.setAccessible(true);
        return field.get(controls.get(sync));
    }

    @Test void pursuitRouteReplansImmediatelyWhenBodyOrEndpointLeavesItsCertificate() {
        var state = new PhysicsState(new Vector3d(10, 70, 0), new Vector3d(1, 0, 0), false);
        var goal = new Vector3d(40, 70, 0);
        var route = new WaypointFlightTarget(java.util.List.of(
                new Vector3d(0, 70, 0), goal), goal, false, false, 2);

        assertFalse(ElytraFlyMovement.needsPursuitReplan(route, goal, state,
                new Vector3d(56, 70, 0)),
                "endpoint drift within the planner freshness bound retains the certified route");
        assertTrue(ElytraFlyMovement.needsPursuitReplan(route, goal,
                new PhysicsState(new Vector3d(10, 70, 5), new Vector3d(1, 0, 0), false),
                new Vector3d(42, 70, 0)), "body outside the certified corridor cannot wait twenty ticks");
        assertTrue(ElytraFlyMovement.needsPursuitReplan(route, goal, state,
                new Vector3d(Math.nextUp(56d), 70, 0)),
                "endpoint drift beyond the planner freshness bound needs a replacement corridor");

        var wideTerminal = new WaypointFlightTarget(java.util.List.of(
                new Vector3d(0, 70, 0), goal), goal, false, false, 8);
        var pastEndpoint = new PhysicsState(new Vector3d(45, 70, 0),
                new Vector3d(1, 0, 0), false);
        assertTrue(wideTerminal.contains(pastEndpoint.position(), pastEndpoint.position()));
        assertTrue(ElytraFlyMovement.needsPursuitReplan(wideTerminal, goal, pastEndpoint,
                new Vector3d(42, 70, 0)),
                "corridor membership cannot retain a terminal waypoint behind the body");

        var elevatedGoal = new Vector3d(40, 90, 0);
        var elevatedRoute = new WaypointFlightTarget(java.util.List.of(
                new Vector3d(0, 70, 0), elevatedGoal), elevatedGoal, false, false, 8);
        assertFalse(ElytraFlyMovement.needsPursuitReplan(elevatedRoute, goal, state,
                new Vector3d(56, 70, 0)),
                "planner-added terrain clearance is not moving-target drift");
        assertTrue(ElytraFlyMovement.needsPursuitReplan(elevatedRoute, goal, state,
                new Vector3d(40, Math.nextUp(86d), 0)),
                "real vertical target drift beyond the freshness bound needs a replacement corridor");
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

    @Test void supportedGroundPursuitPreparesFlightOneLaunchBeforePublishing() throws Exception {
        World world = supportedLaunchWorld();
        var ground = new PhysicsState(new Vector3d(16.5, 64, 8.5), new Vector3d(), true);
        var goal = new Vector3d(60, 64, 8.5);
        var resources = new HazardRecovery.Context(new int[] {1}, 0, false, null);

        var result = LowAltitudeFlightPlan.pursuit(world, ground, goal,
                MovementParameters.DEFAULT, false, resources).get(10, TimeUnit.SECONDS);

        assertNotNull(result, "supported Flight 1 launch must publish both geometry and controls");
        assertTrue(result.route().terrainValidated());
        assertEquals(ground.position(), result.origin());
        assertEquals(goal, result.destination());
        assertNotNull(result.preparedLaunch(), "ground pursuit cannot defer launch controls until airborne");
        assertTrue(result.preparedLaunch().matches(resources));
        assertFalse(result.preparedLaunch().matches(
                new HazardRecovery.Context(new int[] {2}, 0, false, null)),
                "prepared controls retain the Flight 1 lifetime rather than accepting any rocket");
    }

    @Test void alreadyGlidingPursuitWithFlightOneDoesNotPrepareAnotherLaunch() throws Exception {
        World world = supportedLaunchWorld();
        var airborne = new PhysicsState(new Vector3d(16.5, 70, 8.5),
                new Vector3d(.3, 0, 0), false, 0, PlayerPose.GLIDING);
        var resources = new HazardRecovery.Context(new int[] {1}, 0, false, null);

        var result = LowAltitudeFlightPlan.pursuit(world, airborne, new Vector3d(60, 64, 8.5),
                MovementParameters.DEFAULT, true, resources).get(10, TimeUnit.SECONDS);

        assertNotNull(result);
        assertTrue(result.route().terrainValidated());
        assertNull(result.preparedLaunch(), "an in-flight route refresh is not a ground launch");
    }

    @Test void groundPursuitWithoutResourcesRetainsGeometryOnlyCompatibility() throws Exception {
        World world = supportedLaunchWorld();
        var ground = new PhysicsState(new Vector3d(16.5, 64, 8.5), new Vector3d(), true);
        var goal = new Vector3d(60, 64, 8.5);

        var result = LowAltitudeFlightPlan.pursuit(world, ground, goal,
                MovementParameters.DEFAULT, false).get(10, TimeUnit.SECONDS);

        assertNotNull(result, "the resource-free pursuit overload still publishes a geometric route");
        assertTrue(result.route().terrainValidated());
        assertEquals(goal, result.destination());
        assertNull(result.preparedLaunch());
    }

    @Test void groundPursuitRejectsCoolingDownRocketDespiteFeasibleDeploymentAndCorridor() throws Exception {
        assertGroundPursuitLaunchRejected(new HazardRecovery.Context(new int[] {1}, 1, false, null));
    }

    @Test void groundPursuitRejectsAttachedRocketDespiteFeasibleDeploymentAndCorridor() throws Exception {
        assertGroundPursuitLaunchRejected(new HazardRecovery.Context(new int[] {1}, 0, true, null));
    }

    private static void assertGroundPursuitLaunchRejected(HazardRecovery.Context resources) throws Exception {
        World world = supportedLaunchWorld();
        var ground = new PhysicsState(new Vector3d(16.5, 64, 8.5), new Vector3d(), true);
        var goal = new Vector3d(60, 64, 8.5);
        var assessment = DeploymentSafety.assess(world, ground, MovementParameters.DEFAULT, -90, 0);
        assertTrue(assessment.safe(), assessment.reason());
        var deployment = LowAltitudeFlightPlan.deployment(world, ground, MovementParameters.DEFAULT, -90, 0);
        assertNotNull(deployment, "fixture must not reject before the launch control gate");
        var firstGlide = new GlidingPhysics(-90, 0, false, PlayerInput.NONE, MovementParameters.DEFAULT)
                .step(world, deployment, PlayerPose.HALF_WIDTH, PlayerPose.STANDING.height());
        assertFalse(firstGlide.onGround());
        assertTrue(FlightHazards.safeTravel(world, deployment, firstGlide),
                "the first unpowered travel step must reach launch preparation safely");
        assertNotNull(LowAltitudePlanner.planPursuit(world, deployment, goal,
                MovementParameters.DEFAULT, resources), "resources must not hide a missing geometric corridor");

        var future = LowAltitudeFlightPlan.pursuit(world, ground, goal,
                MovementParameters.DEFAULT, false, resources);
        assertNull(future.get(10, TimeUnit.SECONDS), "an unusable rocket cannot publish a ground launch");
        var outcome = assertInstanceOf(LowAltitudeFlightPlan.PlanFuture.class, future);
        assertEquals(LowAltitudeFlightPlan.Status.DEPLOYMENT_REJECTED, outcome.status(),
                "failure must be the launch gate, not a busy worker or swallowed planner exception");
        assertEquals("no complete launch control episode", outcome.detail());
    }

    private static World supportedLaunchWorld() {
        BlockState stone = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 64 ? stone : super.getBlockStateAt(position);
            }
        };
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
