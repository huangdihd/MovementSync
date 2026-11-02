package xin.bbtt.movements;

import java.util.List;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.Entity.Entity;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.EntityFlightTarget;
import xin.bbtt.physics.FlightControlAdmission;
import xin.bbtt.physics.HazardRecovery;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.WaypointFlightTarget;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class ElytraFlyMovementPursuitPredictionTest extends ElytraCommandTestSupport {
    @Test
    void usesDefaultPredictionWithoutRockets() {
        assertEquals(12, ElytraFlyMovement.pursuitPredictionTicks(HazardRecovery.Context.EMPTY));
    }

    @Test
    void preparesTheNextFullEpisodeWhileRocketIsAttached() {
        var resources = new HazardRecovery.Context(new int[] {0}, 0, true, null);

        assertEquals(24, ElytraFlyMovement.pursuitPredictionTicks(resources));
    }

    @Test
    void preparesTheNextFullEpisodeDuringRocketCooldown() {
        var resources = new HazardRecovery.Context(new int[] {0}, 1, false, null);

        assertEquals(24, ElytraFlyMovement.pursuitPredictionTicks(resources));
    }

    @Test
    void predictsThroughFlightZeroRocketLifetime() {
        var resources = new HazardRecovery.Context(new int[] {0}, 0, false, null);

        assertEquals(24, ElytraFlyMovement.pursuitPredictionTicks(resources));
    }

    @Test
    void predictsThroughFlightOneRocketLifetime() {
        var resources = new HazardRecovery.Context(new int[] {1}, 0, false, null);

        assertEquals(34, ElytraFlyMovement.pursuitPredictionTicks(resources));
    }

    @Test
    void deploymentRouteKeepsItsClosingTimeAimAcrossResourceChanges() throws Exception {
        var pursuit = new Pursuit(33);
        var destination = pursuit.target.aim(pursuit.state);
        var currentForecast = WaypointFlightTarget.pursuitAim(pursuit.target, pursuit.state)
                .fma(12, pursuit.target.velocity());
        assertTrue(destination.distance(currentForecast) > 16,
                "closing-time and current fixed-window forecasts must differ beyond the freshness bound");
        var deploymentRoute = new WaypointFlightTarget(
                List.of(pursuit.state.position(), destination), destination, false, false, 8);
        pursuit.movement.withLowAltitudePlan(new LowAltitudeFlightPlan.Result(
                deploymentRoute, pursuit.state.position(), destination, 0, 0));
        setField(pursuit.movement, "lowPlan", new CompletableFuture<LowAltitudeFlightPlan.Result>());
        var source = pursuit.registerFallback();
        setResources(12);

        pursuit.update(pursuit.state);

        assertSame(deploymentRoute, field(pursuit.movement, "lowRoute"));
        assertFalse((boolean) field(pursuit.movement, "lowPursuitRouteStale"));
        assertSame(source, emergencySource());
    }

    @ParameterizedTest
    @ValueSource(ints = {33, 12})
    void publishesCompletedPlanUsingItsOriginalHorizon(int plannedTicks) throws Exception {
        var pursuit = new Pursuit(plannedTicks);
        pursuit.completePlan(pursuit.route);
        setResources(plannedTicks == 33 ? 12 : 33);

        pursuit.update(pursuit.state);

        assertSame(pursuit.route, field(pursuit.movement, "lowRoute"));
        assertEquals(plannedTicks, field(pursuit.movement, "lowRoutePredictionTicks"));
        assertFalse((boolean) field(pursuit.movement, "lowPursuitRouteStale"));
        assertNull(field(pursuit.movement, "lowPlan"), "publication must consume the completed result");
    }

    @ParameterizedTest
    @ValueSource(ints = {33, 12})
    void replacementPlanningUsesCurrentResourcesWithoutChangingAcceptedHorizon(int plannedTicks)
            throws Exception {
        var pursuit = new Pursuit(plannedTicks);
        pursuit.acceptRoute();
        int currentTicks = plannedTicks == 33 ? 12 : 33;
        setResources(currentTicks);
        setField(pursuit.movement, "lowPlan", null);
        setField(pursuit.movement, "lowPlanTicks", 0);

        try {
            pursuit.update(pursuit.state);

            assertNotNull(field(pursuit.movement, "lowPlan"));
            assertEquals(currentTicks == 12 ? 12 : 34, field(pursuit.movement, "lowPlanPredictionTicks"));
            assertEquals(plannedTicks, field(pursuit.movement, "lowRoutePredictionTicks"));
            assertFalse((boolean) field(pursuit.movement, "lowPursuitRouteStale"));
        } finally {
            // The assertion concerns submission, not worker completion or machine throughput.
            if (field(pursuit.movement, "lowPlan") instanceof CompletableFuture<?> plan) plan.cancel(true);
        }
    }

    @ParameterizedTest
    @ValueSource(ints = {33, 12})
    void resourceHorizonSwitchRetainsAcceptedRouteAndFallback(int plannedTicks) throws Exception {
        var pursuit = new Pursuit(plannedTicks);
        pursuit.acceptRoute();
        var source = pursuit.registerFallback();
        setResources(plannedTicks == 33 ? 12 : 33);

        pursuit.update(pursuit.state);

        assertSame(pursuit.route, field(pursuit.movement, "lowRoute"));
        assertFalse((boolean) field(pursuit.movement, "lowPursuitRouteStale"));
        assertSame(source, emergencySource(), "resource-only forecast changes must not revoke fallback");

        pursuit.advanceTarget(1);
        pursuit.update(pursuit.state);
        assertFalse((boolean) field(pursuit.movement, "lowPursuitRouteStale"));
        assertSame(source, emergencySource(), "ordinary .8-block target motion remains fresh");
    }

    @Test
    void rejectsCompletedPlanAfterRealDisplacementEvenWhenHorizonChangeCancelsIt() throws Exception {
        var pursuit = new Pursuit(33);
        pursuit.completePlan(pursuit.route);
        pursuit.advanceTarget(21);
        setResources(12);

        pursuit.update(pursuit.state);

        assertNull(field(pursuit.movement, "lowRoute"),
                "16.8 blocks of real motion cannot be hidden by a 21-tick shorter forecast");
        assertNull(field(pursuit.movement, "lowPlan"));
    }

    @Test
    void revokesAcceptedRouteAfterRealDisplacementEvenWhenHorizonChangeCancelsIt() throws Exception {
        var pursuit = new Pursuit(33);
        pursuit.acceptRoute();
        pursuit.registerFallback();
        pursuit.advanceTarget(21);
        setResources(12);

        pursuit.update(pursuit.state);

        assertSame(pursuit.route, field(pursuit.movement, "lowRoute"));
        assertTrue((boolean) field(pursuit.movement, "lowPursuitRouteStale"));
        assertNull(emergencySource(), "real drift must revoke future fallback rights");
    }

    @ParameterizedTest
    @CsvSource({"false, false", "false, true", "true, false", "true, true"})
    void horizonSwitchPreservesBodyAndPassedEndpointSafeguards(boolean accepted, boolean passed)
            throws Exception {
        var pursuit = new Pursuit(33);
        WaypointFlightTarget route = pursuit.route;
        if (!accepted && !passed) {
            // Returned planner routes must reconcile their certified corridor to the live body.
            var constructor = WaypointFlightTarget.class.getDeclaredConstructor(
                    List.class, Vector3d.class, boolean.class, boolean.class, double.class, World.class);
            constructor.setAccessible(true);
            route = constructor.newInstance(route.waypoints(), route.goal(), false, false, 8,
                    movementSync.getWorld());
            assertTrue(route.terrainValidated());
        }
        var position = passed ? route.goal().add(1, 0, 0) : new Vector3d(10, 70, 9);
        var live = new PhysicsState(position, pursuit.state.velocity(), false);
        assertEquals(passed, route.contains(position, position));
        assertEquals(passed, route.passedEndpoint(position));
        if (accepted) {
            pursuit.acceptRoute();
            pursuit.registerFallback();
        } else {
            pursuit.completePlan(route);
        }
        setResources(12);

        pursuit.update(live);

        if (!accepted) {
            assertNull(field(pursuit.movement, "lowRoute"), "unsafe returned geometry must not publish");
            return;
        }
        assertTrue((boolean) field(pursuit.movement, "lowPursuitRouteStale"));
        assertNull(emergencySource(), "resource stability cannot override body or forward-progress safety");
    }

    @Test
    void altitudeCannotKeepHorizontallyPassedPursuitEndpointCurrent() {
        var start = new Vector3d(71.35, 67.81, 7.8);
        var endpoint = new Vector3d(76.5, 64.6, 8.5);
        var route = new WaypointFlightTarget(List.of(start, endpoint), endpoint, false, false, 8);
        var position = new Vector3d(77.38, 68.69, 8.442);
        var state = new PhysicsState(position, new Vector3d(.4851, .06139, .03254), false);
        assertTrue(route.contains(position, position));
        assertFalse(route.passedEndpoint(position), "3D projection still points behind the descending endpoint");

        assertTrue(ElytraFlyMovement.needsPursuitReplan(route, endpoint, state, endpoint),
                "horizontal target progress must retire the old endpoint despite excess altitude");
    }

    @Test
    void verticalPursuitSegmentKeepsItsAltitudeClearance() {
        var endpoint = new Vector3d(111, 84.7, 10.31);
        var route = new WaypointFlightTarget(List.of(new Vector3d(111, 66.7, 10.31), endpoint),
                endpoint, false, false, 8);
        var state = new PhysicsState(new Vector3d(111.1, 79, 10.31), new Vector3d(0, .5, 0), false);

        assertFalse(ElytraFlyMovement.needsPursuitReplan(route, endpoint, state, endpoint),
                "horizontal progress must not consume a purely vertical climb");
    }

    private void setResources(int predictionTicks) {
        inventory[36] = predictionTicks == 33 ? item("firework_rocket", 1) : null;
        movementSync.inventoryManager.setContainerItems(0, inventory);
    }

    private Object emergencySource() throws Exception {
        return field(field(movementSync, "flightControls"), "emergencySource");
    }

    private static Object field(Object owner, String name) throws Exception {
        var field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(owner);
    }

    private static void setField(Object owner, String name, Object value) throws Exception {
        var field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        field.set(owner, value);
    }

    private final class Pursuit {
        private final PhysicsState state = new PhysicsState(new Vector3d(10, 70, 0),
                new Vector3d(1, 0, 0), false);
        private final Entity entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                80, 70, 0, 0, 0, 0, new Vector3d(.8, 0, 0));
        private final EntityFlightTarget target = new EntityFlightTarget(99, 6,
                entity.getPosition(), entity.getMovement());
        private final ElytraFlyMovement movement = new ElytraFlyMovement(target,
                movementSync.getNavigationGeneration(), ElytraFlyMovement.DEFAULT_ARRIVAL_RADIUS);
        private final WaypointFlightTarget route;
        private final int plannedTicks;

        private Pursuit(int plannedTicks) throws Exception {
            this.plannedTicks = plannedTicks;
            setResources(plannedTicks);
            movementSync.getWorld().getEntities().put(99, entity);
            movementSync.position.set(state.position());
            movementSync.velocity.set(state.velocity());
            movementSync.onGround.set(false);
            movementSync.setGliding(true);
            // Ground target at x=80, six-block following distance, and .8 blocks/tick.
            var destination = new Vector3d(74 + .8 * plannedTicks, 70, 0);
            route = new WaypointFlightTarget(List.of(state.position(), destination),
                    destination, false, false, 8);
            // Keep publication tests off the periodic worker boundary; no planner thread is needed.
            setField(movement, "lowPlanTicks", 1);
            setField(movement, "lowPlanPredictionTicks", plannedTicks);
        }

        private void completePlan(WaypointFlightTarget resultRoute) throws Exception {
            setField(movement, "lowPlan", CompletableFuture.completedFuture(
                    new LowAltitudeFlightPlan.Result(resultRoute, state.position(), route.goal(), 0, 0)));
        }

        private void acceptRoute() throws Exception {
            movement.withLowAltitudePlan(new LowAltitudeFlightPlan.Result(
                    route, state.position(), route.goal(), 0, 0));
            setField(movement, "lowRoutePredictionTicks", plannedTicks);
            setField(movement, "lowPlan", new CompletableFuture<LowAltitudeFlightPlan.Result>());
        }

        private FlightControlAdmission.Source registerFallback() {
            movementSync.claimFlightControlOwner(movement, FlightControlAdmission.Mode.LOCAL_SOLVER);
            var source = FlightControlAdmission.snapshot(movementSync);
            assertTrue(movementSync.registerForwardFlightEmergency(movement, source, -90, route));
            return source;
        }

        private void advanceTarget(int ticks) {
            for (int tick = 0; tick < ticks; tick++) {
                entity.move(new Vector3d(.8, 0, 0));
                target.sample(movementSync.getWorld());
            }
            assertEquals(.8, target.velocity().x, 1.0e-12);
            assertFalse(target.gliding());
        }

        private void update(PhysicsState live) throws Exception {
            var update = ElytraFlyMovement.class.getDeclaredMethod("updateLowRoute",
                    MovementSync.class, PhysicsState.class);
            update.setAccessible(true);
            update.invoke(movement, movementSync, live);
        }
    }
}
