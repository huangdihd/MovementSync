package xin.bbtt.physics;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class KnownReturnPlanPreviewTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1,
            Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0,
            Map.of(), "empty", 0, false, "LAVA");
    private World shore() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d point) {
                if (point.x < 0 && point.y < 40) return STONE;
                if (point.y < 31) return LAVA;
                return super.getBlockStateAt(point);
            }
        };
    }

    @Test void outwardMomentumRequiresActualReversalAndSafeTouchdown() {
        var world = shore();
        var initial = new PhysicsState(new Vector3d(70, 65, .5),
                new Vector3d(1.3, -.1, 0), false, 0, PlayerPose.GLIDING);
        var resources = new HazardRecovery.Context(new int[] {1,1,1,1,1,1,1,1}, 0, false, null);
        var result = KnownReturnPlan.preview(world, initial, MovementParameters.DEFAULT,
                resources, new Vector3d(-24.5, 40, .5), 0);
        assertTrue(result.proven(), result.reason() + " steps=" + result.simulatedSteps());
        var state = initial;
        for (var step : result.steps()) {
            assertEquals(state, step.before());
            var replay = new GlidingPhysics(step.control().yaw(), step.control().pitch(),
                    step.resource().powered(), PlayerInput.NONE, MovementParameters.DEFAULT)
                    .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            state = new PhysicsState(replay.position(), replay.velocity(), replay.onGround(), replay.fallDistance(), replay.pose());
            assertEquals(step.after(), state);
        }
        assertTrue(state.onGround());
        assertTrue(state.position().x < 0);
        assertEquals(40, state.position().y);
        assertFalse(result.steps().get(result.steps().size() - 1).resource().powered());
        assertTrue(result.simulatedSteps() <= KnownReturnPlan.MAX_STEPS);
    }

    @Test void missingKnownSupportCannotBeCertified() {
        var result = KnownReturnPlan.preview(new World(),
                new PhysicsState(new Vector3d(70, 65, .5), new Vector3d(1, 0, 0), false),
                MovementParameters.DEFAULT, HazardRecovery.Context.EMPTY, new Vector3d(-24.5, 40, .5), 0);
        assertFalse(result.proven());
    }
}
