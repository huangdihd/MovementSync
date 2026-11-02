package xin.bbtt.flight;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class RocketPursuitProgressTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(),
            "block", 1.5, true, "STONE");
    private static final World RIDGE = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d position) {
            return position.y < 64 || position.x >= 112 && position.x < 128 && position.y < 84
                    ? STONE : super.getBlockStateAt(position);
        }
    };

    @Test void pursuitDoesNotSpendItsMiddleTicksReturningToTheRidge() {
        var initial = new PhysicsState(new Vector3d(122, 86.38, 8.817),
                new Vector3d(1.136, -.001455, -.0654), false, 1.001455, PlayerPose.GLIDING);
        var routeSource = new PhysicsState(new Vector3d(120.9, 86.38, 8.882),
                new Vector3d(1, 0, 0), false, 1, PlayerPose.GLIDING);
        var route = LowAltitudePlanner.planPursuit(RIDGE, routeSource,
                new Vector3d(158.1, 64.7, 8.5), MovementParameters.DEFAULT);
        assertNotNull(route);
        var tracking = new LocalLookaheadSolver.MovingTarget(new Vector3d(131.7, 64, 8.5),
                new Vector3d(.8, 0, 0));
        var selected = new ElytraFlightController.Decision(-90, 20, true,
                ElytraFlightController.Phase.CRUISE);
        var episode = LocalGotoController.prepareRocketEpisode(RIDGE, MovementParameters.DEFAULT,
                initial, selected, 12, FireworkSchedule.lifetimeBounds(1), tracking.predict(12),
                route, false, tracking);
        assertEquals(27, episode.prefix().size());
        var proof = LocalGotoController.rocketTransitionProof(RIDGE, MovementParameters.DEFAULT,
                initial, episode.prefix(), episode.reserve(), episode.bridge(), tracking.predict(12),
                new HazardRecovery.Context(new int[] {1}, 0, false, null), route);
        assertNotNull(proof);
        for (var branch : proof.branches().subList(1, proof.branches().size())) {
            for (int tick = 1; tick < branch.states().size(); tick++) {
                var state = branch.states().get(tick);
                assertTrue(state.position().distance(tracking.predict(tick + 3)) <= 40,
                        "whole episode, not only its endpoint, must retain pursuit at tick " + tick + ": " + state);
            }
        }
    }

    @Test void completeEpisodeFollowsTheSourceRelativeTargetInsteadOfBrakingAtTheOldEndpoint() {
        var initial = new PhysicsState(new Vector3d(125.4, 87.38, 8.424),
                new Vector3d(1.054, .3253, .1478), false, 1, PlayerPose.GLIDING);
        var routeSource = new PhysicsState(new Vector3d(108.7, 82.97, 8.492),
                new Vector3d(1, 0, 0), false, 1, PlayerPose.GLIDING);
        var route = LowAltitudePlanner.planPursuit(RIDGE, routeSource,
                new Vector3d(126.1, 84.6, 8.5), MovementParameters.DEFAULT);
        assertNotNull(route);
        var tracking = new LocalLookaheadSolver.MovingTarget(new Vector3d(126.1, 64, 8.5),
                new Vector3d(.8, 0, 0));
        var selected = new ElytraFlightController.Decision(-90, 20, true,
                ElytraFlightController.Phase.CRUISE);
        var shortEpisode = LocalGotoController.prepareRocketEpisode(RIDGE, MovementParameters.DEFAULT,
                initial, selected, 7, FireworkSchedule.lifetimeBounds(1), tracking.predict(12),
                route, false, tracking);
        assertTrue(shortEpisode.prefix().isEmpty(),
                "an insufficient old corridor must not turn a pursuit episode into a return flight");
        route = LowAltitudePlanner.planPursuit(RIDGE, initial, tracking.predict(34),
                MovementParameters.DEFAULT);
        assertNotNull(route);
        var episode = LocalGotoController.prepareRocketEpisode(RIDGE, MovementParameters.DEFAULT,
                initial, selected, 7, FireworkSchedule.lifetimeBounds(1), tracking.predict(12),
                route, false, tracking);
        assertEquals(27, episode.prefix().size());
        assertEquals(6, episode.reserve().size());
        assertEquals(1, episode.bridge().size());
        var proof = LocalGotoController.rocketTransitionProof(RIDGE, MovementParameters.DEFAULT,
                initial, episode.prefix(), episode.reserve(), episode.bridge(), tracking.predict(12),
                new HazardRecovery.Context(new int[] {1}, 0, false, null), route);
        assertNotNull(proof);
        assertEquals(25, proof.branches().size());
        for (var branch : proof.branches().subList(1, proof.branches().size())) {
            var states = branch.states();
            for (int tick = 1; tick < states.size(); tick++) {
                var state = states.get(tick);
                assertFalse(state.onGround());
                assertTrue(state.position().distance(tracking.predict(tick)) <= 40,
                        "pursuit lag at source-relative tick " + tick + ": " + state);
            }
            var end = states.get(states.size() - 1);
            assertTrue(end.position().x > initial.position().x + 12,
                    "a moving target must not make a complete episode return to its old endpoint: " + end);
        }
    }
}
