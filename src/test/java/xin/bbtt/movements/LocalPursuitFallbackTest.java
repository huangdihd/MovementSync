package xin.bbtt.movements;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.flight.FlightContext;
import xin.bbtt.flight.FlightContextBuilder;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class LocalPursuitFallbackTest {
    private static class Terrain extends World {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public boolean isBoxColliding(double x, double y, double z,
                double X, double Y, double Z) { return y < 0; }
    }

    private FlightContext context(World world, int rockets) {
        return new FlightContext(world,
                new PhysicsState(new Vector3d(8, 5, 8), new Vector3d(0, 0, .1), false),
                MovementParameters.DEFAULT, DamageParameters.NONE, 20,
                new Vector3d(8, 0, 80), 0, 0, rockets, false, 2, false, Map.of());
    }

    @Test void terminalAirCaptureRequiresCompleteVisibleApproach() {
        var initial = new PhysicsState(new Vector3d(8, 5, 8), new Vector3d(0, 0, .8),
                false, 0, PlayerPose.GLIDING);
        var goal = new Vector3d(8, 5, 12);
        var points = java.util.List.of(initial.position(), goal);
        var route = new WaypointFlightTarget(points, goal, false, false, 4);
        assertTrue(ElytraFlyMovement.canCaptureAirborneTarget(new Terrain(), initial, route, goal));
        assertFalse(ElytraFlyMovement.canCaptureAirborneTarget(new Terrain(), initial,
                new WaypointFlightTarget(points, goal, false, true, 4), goal));
        var distant = new Vector3d(8, 5, 45);
        assertFalse(ElytraFlyMovement.canCaptureAirborneTarget(new Terrain(), initial,
                new WaypointFlightTarget(java.util.List.of(initial.position(), distant), distant, false), distant));
        World unknown = new Terrain() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
        };
        World wall = new Terrain() {
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return Z >= 10; }
        };
        assertFalse(ElytraFlyMovement.canCaptureAirborneTarget(unknown, initial, route, goal));
        assertFalse(ElytraFlyMovement.canCaptureAirborneTarget(wall, initial, route, goal));
    }

    @Test void missingRouteCanUseValidatedRocketWithoutDescendingOntoGroundTarget() {
        var config = new FlightContextBuilder().addStandard().build();
        var context = context(new Terrain(), 4);
        var decision = ElytraFlyMovement.localPursuitDecision(context, context.target(), config);
        assertNotNull(decision);
        assertTrue(decision.firework());
        assertNotEquals(ElytraFlightController.Phase.FORCED_LANDING, decision.phase());
        assertNotNull(ElytraFlyMovement.airbornePursuitPreview(context, decision, config));
        var unpowered = context(new Terrain(), 0);
        var coast = ElytraFlyMovement.localPursuitDecision(unpowered, unpowered.target(), config);
        assertNotNull(coast);
        assertFalse(coast.firework());
    }

    @Test void bridgeAboveGroundTargetDoesNotTradeHorizontalProgressForRunawayClimb() {
        var configuration = new FlightContextBuilder().addStandard().build();
        var initial = new PhysicsState(new Vector3d(125, 96, 8.5),
                new Vector3d(.3, .3, 0), false, 0, PlayerPose.GLIDING);
        var aim = new Vector3d(140, 64, 8.5);
        var context = new FlightContext(new Terrain(), initial,
                MovementParameters.DEFAULT, DamageParameters.NONE, 20,
                aim, -90, -20, 0, true, 2, false, Map.of());

        var decision = ElytraFlyMovement.localPursuitDecision(context, aim, configuration);

        assertNotNull(decision);
        assertFalse(decision.firework());
        var end = ElytraFlyMovement.airbornePursuitPreview(context, decision, configuration);
        assertNotNull(end);
        assertTrue(end.position().distance(aim) < initial.position().distance(aim),
                () -> "bridge must close three-dimensional pursuit distance: " + decision + " -> " + end);
        assertTrue(end.position().y < initial.position().y,
                "a bridge already far above a ground target must descend, not climb farther");
    }

    @Test void unknownFireWaterAndBlockedSweepsCannotBecomeLocalRoutes() {
        var config = new FlightContextBuilder().addStandard().build();
        World unknown = new Terrain() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
        };
        World wall = new Terrain() {
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return true; }
        };
        for (World world : new World[] {unknown, wall, liquid("water"), liquid("fire"), liquid("lava")}) {
            var context = context(world, 4);
            assertNull(ElytraFlyMovement.localPursuitDecision(context, context.target(), config));
        }
    }

    @Test void continuationRejectsFloorEvenWhenFirstSixTicksAreAirborne() {
        var config = new FlightContextBuilder().addStandard().build();
        var context = context(new Terrain(), 0);
        var dive = new ElytraFlightController.Decision(0, 80, false, ElytraFlightController.Phase.CRUISE);
        var physics = new GlidingPhysics(0, 80, false);
        var state = context.state();
        for (int tick = 0; tick < 6; tick++) {
            var result = physics.step(context.world(), state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            assertTrue(result.position().y > 0);
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance(), result.pose());
        }
        assertNull(ElytraFlyMovement.airbornePursuitPreview(context, dive, config));
    }

    @Test void registeredDamageRulesAlsoRestrictLocalPursuit() {
        var context = context(new Terrain(), 4);
        var standard = new FlightContextBuilder().addStandard().build();
        var config = new xin.bbtt.flight.FlightConfiguration(standard.modes(), standard.costs(),
                java.util.List.of((c, action, before, after) -> new xin.bbtt.flight.DamageRule.Damage(1, 1)), false);
        assertNull(ElytraFlyMovement.localPursuitDecision(context, context.target(), config));
    }

    private World liquid(String block) {
        return new Terrain() {
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return new BlockState(block, 0, Map.of(), "empty", 0, false, "");
            }
        };
    }
}
