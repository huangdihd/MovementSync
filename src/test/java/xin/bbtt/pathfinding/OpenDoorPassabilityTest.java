package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.world.World;

/**
 * Doors with open=true must be walkable: the pathfinder treats an open door
 * cell as passable, and a closed one as an obstacle.
 */
class OpenDoorPassabilityTest {
    private static final int AIR_ID = 0;
    private static final int STONE_ID = 1;
    private static final int OPEN_NORTH_LEFT_JUNGLE_DOOR_ID = 13995;

    private static BlockState state(String name, Map<String, String> properties) {
        return new BlockState(name, 0, properties, "block", 1.0, true, "wood");
    }

    @Test
    void openDoorIsPassableAndClosedDoorIsNot() {
        assertTrue(state("jungle_door", Map.of("open", "true", "half", "lower")).isPassable());
        assertFalse(state("jungle_door", Map.of("open", "false", "half", "lower")).isPassable());
        assertTrue(state("spruce_trapdoor", Map.of("open", "true")).isPassable());
        assertTrue(state("oak_fence_gate", Map.of("open", "true")).isPassable());
        assertFalse(state("barrel", Map.of("open", "true")).isPassable(),
            "an inventory open state must not turn a full-cube barrel into a path opening");
    }

    @Test
    void pathfinderWalksThroughOpenDoorInClosedWall() {
        World world = new World() {
            private BlockState at(int x, int y, int z) {
                if (y < 70 || y > 71) return BlockStateParser.Instance.parseStateId(AIR_ID);
                if (y == 70) return BlockStateParser.Instance.parseStateId(STONE_ID);
                if (z == 998 && x >= 973 && x <= 979) {
                    if (x == 976) {
                        return BlockStateParser.Instance.parseStateId(OPEN_NORTH_LEFT_JUNGLE_DOOR_ID);
                    }
                    return BlockStateParser.Instance.parseStateId(STONE_ID);
                }
                return BlockStateParser.Instance.parseStateId(AIR_ID);
            }

            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public BlockState getBlockStateAt(org.joml.Vector3d position) {
                return at((int) Math.floor(position.x), (int) Math.floor(position.y), (int) Math.floor(position.z));
            }
        };

        Node start = new Node(976, 71, 996);
        Node goal = new Node(976, 71, 1000);
        List<PathStep> path = new DStarLite(start, goal, new DefaultPathfindingContext(world, false)).findPath(2000);

        assertFalse(path == null || path.isEmpty(), "open door cell must be routable");
        assertTrue(goal.equals(path.get(path.size() - 1).getNode()), "path must reach the room interior");
    }
}
