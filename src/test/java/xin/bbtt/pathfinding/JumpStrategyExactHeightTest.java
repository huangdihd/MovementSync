package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

final class JumpStrategyExactHeightTest {
    private static final int AIR = 0;
    private static final int STONE = 1;
    private static final int WHITE_CARPET = 12694;
    private static final int TOP_OAK_SLAB = 13129;

    @Test
    void rejectsIntegerAdjacentNodeWhoseExactRiseExceedsJumpHeight() {
        World world = worldWithSupports(WHITE_CARPET, STONE);

        List<Edge> edges = new JumpStrategy(false).findEdges(new Node(0, 1, 0), world);

        assertFalse(edges.stream().anyMatch(edge -> edge.getTarget().equals(new Node(1, 2, 0))),
            "carpet top 0.0625 to raised stone top 2.0 is not a reachable jump");
    }

    @Test
    void keepsOneBlockExactJump() {
        World world = worldWithSupports(STONE, STONE);

        List<Edge> edges = new JumpStrategy(false).findEdges(new Node(0, 1, 0), world);

        assertTrue(edges.stream().anyMatch(edge -> edge.getTarget().equals(new Node(1, 2, 0))));
    }

    @Test
    void walkRejectsSameNodeRiseAboveStepHeight() {
        World world = worldWithSupportsAtLevels(WHITE_CARPET, 0, TOP_OAK_SLAB, 0);

        List<Edge> edges = new WalkStrategy(false).findEdges(new Node(0, 1, 0), world);

        assertFalse(edges.stream().anyMatch(edge -> edge.getTarget().equals(new Node(1, 1, 0))),
            "carpet top 0.0625 to top-slab top 1.0 exceeds the 0.6 step limit");
    }

    @Test
    void fallUsesExactDropRatherThanIntegerNodeDifference() {
        World world = worldWithSupportsAtLevels(STONE, 0, WHITE_CARPET, -1);

        List<Edge> edges = new FallStrategy(1).findEdges(new Node(0, 1, 0), world);

        assertFalse(edges.stream().anyMatch(edge -> edge.getTarget().equals(new Node(1, 0, 0))),
            "stone top 1.0 to carpet top -0.9375 exceeds a one-block fall limit");
    }

    @Test
    void walkDigFallbackKeepsJumpReachableSupportAboveStepHeight() {
        World world = worldWithObstacle(WHITE_CARPET, 0, STONE, 0, 1);

        List<Edge> edges = new WalkStrategy(true).findEdges(new Node(0, 1, 0), world);

        assertTrue(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.DIG
                && edge.getTarget().equals(new Node(1, 1, 0))));
    }

    @Test
    void walkDigFallbackRejectsJumpWhenSourceOverheadIsBlocked() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return AIR;
                if (x == 0 && y == 0) return WHITE_CARPET;
                if (x == 0 && y == 2) return STONE;
                if (x == 1 && (y == 0 || y == 1)) return STONE;
                return AIR;
            }
        };

        List<Edge> edges = new WalkStrategy(true).findEdges(new Node(0, 1, 0), world);

        assertFalse(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.DIG
                && edge.getTarget().equals(new Node(1, 1, 0))));
    }

    @Test
    void jumpDigFallbackRejectsSupportAboveJumpHeight() {
        World world = worldWithObstacle(WHITE_CARPET, 0, STONE, 1, 2);

        List<Edge> edges = new JumpStrategy(true).findEdges(new Node(0, 1, 0), world);

        assertFalse(edges.stream().anyMatch(edge ->
            edge.getType() == BuiltinMovementType.DIG
                && edge.getTarget().equals(new Node(1, 2, 0))));
    }

    private static World worldWithSupports(int sourceState, int targetState) {
        return worldWithSupportsAtLevels(sourceState, 0, targetState, 1);
    }

    private static World worldWithSupportsAtLevels(
            int sourceState, int sourceY, int targetState, int targetY) {
        return new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return AIR;
                if (x == 0 && y == sourceY) return sourceState;
                if (x == 1 && y == targetY) return targetState;
                return AIR;
            }
        };
    }

    private static World worldWithObstacle(
            int sourceState, int sourceY, int supportState, int supportY, int obstacleY) {
        return new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return AIR;
                if (x == 0 && y == sourceY) return sourceState;
                if (x == 1 && y == supportY) return supportState;
                if (x == 1 && y == obstacleY) return STONE;
                return AIR;
            }
        };
    }
}
