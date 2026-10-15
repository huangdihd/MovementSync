package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.physics.BlockMovementProperties;
import xin.bbtt.physics.JumpInputController;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class IceArrivalControllerTest {
    @Test
    void iceAndBlueIceSettleWithoutEndlessCenterCrossings() {
        for (String name : new String[]{"ice", "packed_ice", "blue_ice"}) {
            int block = stateId(name);
            World world = new World() {
                @Override public boolean chunkLoaded(int x, int z) { return true; }
                @Override public int getBlockAt(Vector3d position) { return position.y < 0 ? block : 0; }
            };
            double slipperiness = BlockMovementProperties.of(BlockStateParser.Instance.parseStateId(block)).slipperiness();
            Vector3d target = new Vector3d(3.5, 0, 0.5);
            PhysicsState state = new PhysicsState(new Vector3d(0.5, 0, 0.5), new Vector3d(0.2, 0, 0), true);
            int crossings = 0;
            boolean settled = false;
            for (int tick = 0; tick < 200; tick++) {
                var command = JumpInputController.groundArrival(state.position(), target, state.velocity(), slipperiness);
                var result = new WalkingPhysics(command.input(), command.yaw()).step(world, state,
                        StandablePositionResolver.PLAYER_HALF_WIDTH, StandablePositionResolver.PLAYER_HEIGHT);
                if ((state.position().x - target.x) * (result.position().x - target.x) < 0) crossings++;
                state = new PhysicsState(result.position(), result.velocity(), result.onGround());
                if (Math.abs(state.position().x - target.x) < 0.3
                        && Math.hypot(state.velocity().x, state.velocity().z) < 0.025) {
                    settled = true;
                    break;
                }
            }
            assertTrue(settled, name + " must settle using keys, not velocity edits");
            assertTrue(crossings <= 2, name + " crossed target " + crossings + " times");
        }
    }

    private static int stateId(String name) {
        for (int id = 0; id < BlockStateParser.getBlockStateRegistrySize(); id++) {
            if (name.equals(BlockStateParser.Instance.parseStateId(id).blockName())) return id;
        }
        throw new AssertionError("Missing block: " + name);
    }
}
