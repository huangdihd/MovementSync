package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class TraversalSafetyTest {
    @Test
    void trapsAtFeetAndHeadNeverBecomeWalkingOrDigDestinations() {
        for (String name : new String[]{"cobweb", "sweet_berry_bush", "powder_snow", "lava"}) {
            for (int height : new int[]{0, 1}) {
                World world = world(name, 1, height, 0);
                Node source = new Node(0, 0, 0);
                Node target = new Node(1, 0, 0);
                assertTrue(StandablePositionResolver.feetY(world, 1, 0, 0).isEmpty(), name);
                assertFalse(StandablePositionResolver.canWalk(world, source, target), name);
                assertFalse(StandablePositionResolver.canWalkToSupport(world, source, target), name);
                assertFalse(StandablePositionResolver.canJumpToSupport(world, source, target), name);
                assertFalse(new WalkStrategy(true).findEdges(source, world).stream()
                        .anyMatch(edge -> edge.getTarget().equals(target)), name);
            }
        }
    }

    @Test
    void diagonalCornerAndJumpArcCannotClipTrap() {
        World corner = world("cobweb", 1, 0, 0);
        assertTrue(StandablePositionResolver.feetY(corner, 1, 0, 1).isPresent());
        assertFalse(StandablePositionResolver.canWalk(corner, new Node(0, 0, 0), new Node(1, 0, 1)));
        World arc = world("sweet_berry_bush", 1, 1, 0);
        assertFalse(StandablePositionResolver.canGapJump(arc, new Node(0, 0, 0), new Node(3, 0, 0)));
    }

    private static World world(String name, int hazardX, int hazardY, int hazardZ) {
        int hazard = stateId(name);
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (y < 0) return 1;
                return x == hazardX && y == hazardY && z == hazardZ ? hazard : 0;
            }
        };
    }

    private static int stateId(String name) {
        for (int id = 0; id < BlockStateParser.getBlockStateRegistrySize(); id++) {
            if (name.equals(BlockStateParser.Instance.parseStateId(id).blockName())) return id;
        }
        throw new AssertionError("Missing block: " + name);
    }
}
