package xin.bbtt.movements;

import java.util.List;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.Entity.Entity;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.EntityFlightTarget;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.WaypointFlightTarget;

import static org.junit.jupiter.api.Assertions.*;

final class LowRouteRefreshCadenceTest extends ElytraCommandTestSupport {
    @ParameterizedTest
    @ValueSource(ints = {19, 20, 40})
    void acceptedRouteRestartsPeriodicRefreshAtDeploymentPhase(int previousTicks) throws Exception {
        try (var pursuit = new Pursuit()) {
            setField(pursuit.movement, "lowPlanTicks", previousTicks);
            pursuit.completePlan(pursuit.destination);

            pursuit.update(pursuit.state);

            assertSame(pursuit.route, field(pursuit.movement, "lowRoute"));
            assertFalse((boolean) field(pursuit.movement, "lowPursuitRouteStale"));
            assertNull(field(pursuit.movement, "lowPlan"),
                    "acceptance must not immediately submit another periodic plan");
            // Publication is the first eligible evaluation, matching withLowAltitudePlan's phase.
            for (int evaluation = 2; evaluation < 20; evaluation++) {
                pursuit.update(pursuit.state);
                assertNull(field(pursuit.movement, "lowPlan"),
                        "fresh geometry must survive evaluation " + evaluation);
                assertSame(pursuit.route, field(pursuit.movement, "lowRoute"));
            }

            pursuit.update(pursuit.state);

            assertNotNull(field(pursuit.movement, "lowPlan"),
                    "resetting the cycle must preserve the existing periodic refresh");
            assertFalse((boolean) field(pursuit.movement, "lowPursuitRouteStale"));
        }
    }

    private enum Staleness { TARGET_DRIFT, BODY_DRIFT, PASSED_ENDPOINT }

    @ParameterizedTest
    @EnumSource(Staleness.class)
    void realStalenessReplansImmediatelyAfterAcceptance(Staleness staleness) throws Exception {
        try (var pursuit = new Pursuit()) {
            setField(pursuit.movement, "lowPlanTicks", 20);
            pursuit.completePlan(pursuit.destination);
            pursuit.update(pursuit.state);
            assertSame(pursuit.route, field(pursuit.movement, "lowRoute"));
            assertNull(field(pursuit.movement, "lowPlan"));

            PhysicsState live = pursuit.state;
            switch (staleness) {
                case TARGET_DRIFT -> {
                    pursuit.entity.move(new Vector3d(17, 0, 0));
                    pursuit.target.sample(movementSync.getWorld());
                }
                case BODY_DRIFT -> live = new PhysicsState(
                        pursuit.state.position().add(0, 0, 9), pursuit.state.velocity(), false);
                case PASSED_ENDPOINT -> live = new PhysicsState(
                        new Vector3d(pursuit.destination).add(1, 0, 0), pursuit.state.velocity(), false);
            }

            pursuit.update(live);

            assertTrue((boolean) field(pursuit.movement, "lowPursuitRouteStale"));
            assertNotNull(field(pursuit.movement, "lowPlan"),
                    "a fresh periodic cycle must not postpone " + staleness);
        }
    }

    @ParameterizedTest
    @ValueSource(booleans = {false, true})
    void failedOrRejectedResultDoesNotRestartPeriodicRefresh(boolean staleResult) throws Exception {
        try (var pursuit = new Pursuit()) {
            setField(pursuit.movement, "lowPlanTicks", 19);
            if (staleResult) pursuit.completePlan(new Vector3d(pursuit.destination).add(17, 0, 0));
            else setField(pursuit.movement, "lowPlan", CompletableFuture.completedFuture(null));

            pursuit.update(pursuit.state);

            assertNull(field(pursuit.movement, "lowRoute"), "only a fresh result may be accepted");
            assertNull(field(pursuit.movement, "lowPlan"));

            pursuit.update(pursuit.state);

            assertNull(field(pursuit.movement, "lowRoute"));
            assertNotNull(field(pursuit.movement, "lowPlan"),
                    "failure or rejection must not delay the pending periodic refresh");
        }
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

    private final class Pursuit implements AutoCloseable {
        private final PhysicsState state = new PhysicsState(new Vector3d(10, 70, 0),
                new Vector3d(1, 0, 0), false);
        private final Entity entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                80, 70, 0, 0, 0, 0, new Vector3d());
        private final EntityFlightTarget target = new EntityFlightTarget(99, 6, entity.getPosition());
        private final ElytraFlyMovement movement = new ElytraFlyMovement(target,
                movementSync.getNavigationGeneration(), ElytraFlyMovement.DEFAULT_ARRIVAL_RADIUS);
        private final Vector3d destination = target.position();
        private final WaypointFlightTarget route = new WaypointFlightTarget(
                List.of(state.position(), destination), destination, false, false, 8);

        private Pursuit() {
            movementSync.getWorld().getEntities().put(99, entity);
            movementSync.position.set(state.position());
            movementSync.velocity.set(state.velocity());
            movementSync.onGround.set(false);
            movementSync.setGliding(true);
        }

        private void completePlan(Vector3d resultDestination) throws Exception {
            setField(movement, "lowPlan", CompletableFuture.completedFuture(
                    new LowAltitudeFlightPlan.Result(route, state.position(), resultDestination, 0, 0)));
        }

        private void update(PhysicsState live) throws Exception {
            var update = ElytraFlyMovement.class.getDeclaredMethod("updateLowRoute",
                    MovementSync.class, PhysicsState.class);
            update.setAccessible(true);
            update.invoke(movement, movementSync, live);
        }

        @Override
        public void close() throws Exception {
            // Assert submission only; never depend on planner throughput or completion timing.
            if (field(movement, "lowPlan") instanceof CompletableFuture<?> plan) plan.cancel(true);
        }
    }
}
