package xin.bbtt.flight;

import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class VerticalPursuitProgressTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(),
            "block", 1.5, true, "STONE");
    private static final World RIDGE = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d position) {
            return position.y < 64 || position.x >= 112 && position.x < 128 && position.y < 84
                    ? STONE : super.getBlockStateAt(position);
        }
    };

    @Test void descendingRouteDoesNotRewardClimbingAwayFromTheMovingTarget() {
        var initial = new PhysicsState(new Vector3d(103, 98.15, 8.371),
                new Vector3d(.07475, .3555, -.00903), false, 1, PlayerPose.GLIDING);
        var routeSource = new PhysicsState(new Vector3d(102.9, 97.80, 8.380),
                new Vector3d(.07811, .4262, -.01018), false, 1, PlayerPose.GLIDING);
        var route = LowAltitudePlanner.planPursuit(RIDGE, routeSource,
                new Vector3d(134.9, 64, 8.5), MovementParameters.DEFAULT);
        assertNotNull(route);
        // The logged goal is the twelve-tick prediction. The source observation
        // is earlier, and the real entity is eight blocks ahead of its follow aim.
        var tracking = new LocalLookaheadSolver.MovingTarget(new Vector3d(108.5, 64, 8.5),
                new Vector3d(.8, 0, 0));
        var selected = new ElytraFlightController.Decision(-89.52011f, 20, true,
                ElytraFlightController.Phase.CRUISE);
        var episode = LocalGotoController.prepareRocketEpisode(RIDGE, MovementParameters.DEFAULT,
                initial, selected, 12, FireworkSchedule.lifetimeBounds(1), tracking.predict(12),
                route, false, tracking);
        assertEquals(27, episode.prefix().size());
        assertEquals(6, episode.reserve().size());
        assertEquals(1, episode.bridge().size());

        var live = initial;
        for (int tick = 0; tick < 2; tick++) {
            var result = new GlidingPhysics(270.4799f, -70, false).step(RIDGE, live,
                    PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            live = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                    result.fallDistance(), PlayerPose.GLIDING);
        }
        for (var replayInitial : List.of(initial, live)) {
            int delay = replayInitial == initial ? 0 : 2;
            var proof = LocalGotoController.rocketTransitionProof(RIDGE, MovementParameters.DEFAULT,
                    replayInitial, episode.prefix(), episode.reserve(), episode.bridge(),
                    tracking.predict(12), new HazardRecovery.Context(new int[] {1}, 0, false, null), route);
            assertNotNull(proof, "source and delayed publication need independent all-outcome proofs");
            assertEquals(25, proof.branches().size());
            for (var branch : proof.branches().subList(1, proof.branches().size())) {
                for (int tick = 1; tick < branch.states().size(); tick++) {
                    var state = branch.states().get(tick);
                    assertFalse(state.onGround());
                    assertTrue(state.position().distance(tracking.predict(tick + delay).add(8, 0, 0)) <= 40,
                            "pursuit must remain bounded throughout the episode: delay=" + delay
                                    + " tick=" + tick + " state=" + state);
                }
            }
        }
    }
}
