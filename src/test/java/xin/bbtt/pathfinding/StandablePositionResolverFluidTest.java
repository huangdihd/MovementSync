package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.*;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.world.World;

final class StandablePositionResolverFluidTest {
    @Test
    void lavaAtFeetOrHeadCannotBeAStandableDestinationOrHypotheticalDigSource() {
        for (int lavaY : new int[]{0, 1}) {
            World world = fluidColumn("lava", lavaY, lavaY);
            Node wet = new Node(0, 0, 0);
            Node dry = new Node(1, 0, 0);
            assertTrue(StandablePositionResolver.feetY(world, 0, 0, 0).isEmpty());
            assertTrue(StandablePositionResolver.feetY(world, 1, 0, 0).isPresent());
            assertFalse(StandablePositionResolver.canWalk(world, dry, wet));
            assertFalse(StandablePositionResolver.canWalk(world, wet, dry));
            assertFalse(StandablePositionResolver.canJump(world, dry, wet));
            assertFalse(StandablePositionResolver.canWalkToSupport(world, dry, wet));
            assertFalse(StandablePositionResolver.canJumpToSupport(world, dry, wet));
            assertFalse(StandablePositionResolver.canFall(world, new Node(1, 1, 0), wet, 3));
        }
    }

    @Test
    void deepWaterFloorCannotBecomeGroundDestinationOrDigSource() {
        World world = fluidColumn("water", 0, 3);
        Node wet = new Node(0, 0, 0);
        Node dry = new Node(1, 0, 0);
        assertTrue(StandablePositionResolver.feetY(world, 0, 0, 0).isEmpty());
        assertFalse(StandablePositionResolver.canWalk(world, dry, wet));
        assertFalse(StandablePositionResolver.canWalk(world, wet, dry));
        assertFalse(StandablePositionResolver.canFall(world, new Node(1, 1, 0), wet, 3));
    }

    @Test
    void fluidLavaRouteNeverEntersTrenchOrReturnsReportedUnsafeNearestNode() {
        int lava = stateId("lava");
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (y < -61) return 1;
                if (y == -61) return x == 63600 && z >= 6005 && z <= 6012 ? lava : 1;
                if (x >= 63597 && x <= 63603 && z >= 5998 && z <= 6020 && y <= -56) {
                    return x == 63600 && z >= 5999 && z <= 6019 ? 0 : 1;
                }
                return 0;
            }
        };
        for (int z = 6005; z <= 6012; z++) {
            assertTrue(StandablePositionResolver.feetY(world, 63600, -61, z).isEmpty());
        }
        var context = new PathfindingContextBuilder(world).addNoPlacementStrategies().build();
        var path = new DStarLite(new Node(63600, -60, 6000), new Node(63600, -60, 6017), context)
                .findPath(200);
        assertFalse(path.isEmpty());
        for (var step : path) {
            Node node = step.getNode();
            assertFalse(node.x == 63600 && node.z >= 6005 && node.z <= 6012 && node.y < -60,
                    "Path enters lava: " + node);
        }
        assertNotEquals(new Node(63600, -61, 6012), path.get(path.size() - 1).getNode());
    }

    private static World fluidColumn(String name, int bottom, int top) {
        int fluid = stateId(name);
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                if (position.y < 0) return 1;
                return (int) Math.floor(position.x) == 0 && (int) Math.floor(position.z) == 0
                        && position.y >= bottom && position.y < top + 1 ? fluid : 0;
            }
        };
    }

    private static int stateId(String name) {
        for (int id = 0; id < BlockStateParser.getBlockStateRegistrySize(); id++) {
            var state = BlockStateParser.Instance.parseStateId(id);
            if (state.blockName().equals(name) && "0".equals(state.getProperty("level"))) return id;
        }
        throw new AssertionError("Missing fluid state: " + name);
    }
}
