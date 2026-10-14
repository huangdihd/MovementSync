package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.*;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

final class EnvironmentalDefaultStrategyTest {
    @org.junit.jupiter.api.BeforeEach
    void initializeSingleton() { new xin.bbtt.MovementSync(); }

    @org.junit.jupiter.api.AfterEach
    void resetSingleton() { xin.bbtt.MovementSync.INSTANCE = null; }

    @Test
    void ladderEndingBelowLandingCannotBorrowAnImaginaryPlacedSupport() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (y < 1 || x == 1 && y < 6) return 1;
                return x == 0 && z == 0 && y <= 3 ? 5519 : 0;
            }
        };
        var context = new PathfindingContextBuilder(world).addDefaultStrategies().build();
        Node goal = new Node(1, 6, 0);
        var path = new DStarLite(new Node(0, 1, 0), goal, context).findPath(200);
        assertFalse(path.isEmpty());
        assertNotEquals(goal, path.get(path.size() - 1).getNode());
        assertTrue(context.getEdges(new Node(0, 3, 0)).stream()
                .noneMatch(edge -> edge.getType() == BuiltinMovementType.WALK
                        || edge.getType() == BuiltinMovementType.JUMP));
    }

    @Test
    void submergedFloorIsNotAWalkingOrFallingDestination() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                if (position.y < 1) return 1;
                return position.y < 4 ? 86 : 0;
            }
        };
        assertTrue(StandablePositionResolver.feetY(world, 0, 1, 0).isEmpty());
        var edges = new PathfindingContextBuilder(world).addDefaultStrategies().build()
                .getEdges(new Node(0, 1, 0));
        assertEquals(1, edges.size());
        assertEquals(new Node(0, 2, 0), edges.get(0).getTarget());
        assertEquals(BuiltinMovementType.SWIM, edges.get(0).getType());
    }
}
