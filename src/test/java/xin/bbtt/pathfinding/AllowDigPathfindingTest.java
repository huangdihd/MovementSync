package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;

/**
 * Digging edges must only exist when the caller explicitly allows digging.
 * A pathfinder must not silently mine walls, doors, or terrain when a caller
 * requested non-destructive navigation.
 */
final class AllowDigPathfindingTest {
    private static final BlockState SOLID = new BlockState(
        "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
        "minecraft:air", 0, Map.of(), "empty", 0.0, false, "AIR");

    private final World world = new World() {
        @Override
        public boolean chunkLoaded(int chunkX, int chunkZ) {
            return true;
        }

        @Override
        public BlockState getBlockStateAt(Vector3d position) {
            int x = (int) Math.floor(position.x);
            int y = (int) Math.floor(position.y);
            int z = (int) Math.floor(position.z);
            if (y == 63) return SOLID;                             // infinite floor
            if (x == 1 && (y == 64 || y == 65)) return SOLID;      // 2-high wall, too tall to jump
            return AIR;
        }
    };


    @Test
    void walkStrategyWithoutDiggingNeverEmitsDigEdges() {
        PathfindingContext context = new PathfindingContextBuilder(world)
            .addWalk(false)
            .build();

        List<Edge> edges = context.getEdges(new Node(0, 64, 0));
        assertTrue(edges.stream().noneMatch(edge -> edge.getType() == BuiltinMovementType.DIG),
            "no-dig walk strategy must not emit dig edges: " + edges);
        assertTrue(edges.stream().anyMatch(edge -> edge.getType() == BuiltinMovementType.WALK),
            "plain walk edges must remain available");
    }

    @Test
    void walkStrategyWithDiggingKeepsDigEdges() {
        PathfindingContext context = new PathfindingContextBuilder(world)
            .addWalk(true)
            .build();

        List<Edge> edges = context.getEdges(new Node(0, 64, 0));
        assertTrue(edges.stream().anyMatch(edge -> edge.getType() == BuiltinMovementType.DIG),
            "dig-allowed walk strategy must keep dig edges: " + edges);
    }

    @Test
    void defaultWalkStrategyIsFailClosed() {
        PathfindingContext context = new PathfindingContextBuilder(world)
            .addWalk()
            .build();

        List<Edge> edges = context.getEdges(new Node(0, 64, 0));
        assertTrue(edges.stream().noneMatch(edge -> edge.getType() == BuiltinMovementType.DIG),
            "callers must opt in explicitly before a walk strategy may emit dig edges");
    }

    @Test
    void defaultContextIsFailClosedAndExplicitContextCanDig() {
        PathfindingContext noDig = new DefaultPathfindingContext(world);
        assertTrue(noDig.getEdges(new Node(0, 64, 0)).stream()
                .noneMatch(edge -> edge.getType() == BuiltinMovementType.DIG),
            "the default context must not emit destructive edges");

        PathfindingContext digAllowed = new DefaultPathfindingContext(world, true);
        assertTrue(digAllowed.getEdges(new Node(0, 64, 0)).stream()
                .anyMatch(edge -> edge.getType() == BuiltinMovementType.DIG),
            "only an explicit path-scoped opt-in may restore dig edges");
    }

    @Test
    void wallStopsPlannerWhenDiggingDisabled() {
        Node start = new Node(0, 64, 0);
        Node goal = new Node(2, 64, 0);

        List<PathStep> path = new DStarLite(start, goal, new DefaultPathfindingContext(world)).findPath(2000);
        assertTrue(path.isEmpty() || !goal.equals(path.get(path.size() - 1).getNode()),
            "the wall must stop a no-dig planner from reaching the goal: " + path);
    }

    @Test
    void plannerGainsDigEdgeThroughWallWhenDiggingEnabled() {
        List<Edge> edges = new DefaultPathfindingContext(world, true).getEdges(new Node(0, 64, 0));
        assertTrue(edges.stream().anyMatch(edge ->
                edge.getType() == BuiltinMovementType.DIG && edge.getTarget().equals(new Node(1, 64, 0))),
            "with digging the planner may tunnel into the wall: " + edges);
    }

    @Test
    void digEdgeAcceptsExactNonSolidSupportBelowObstacle() {
        World exactSupportWorld = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return 0;
                if (x == 0 && y == 0) return 1;
                if (x == 1 && y == 0) return 6721; // four snow layers, top at 0.5
                if (x == 1 && y == 1) return 1;    // obstacle to dig
                return 0;
            }
        };

        List<Edge> edges = new WalkStrategy(true).findEdges(new Node(0, 1, 0), exactSupportWorld);

        assertTrue(edges.stream().anyMatch(edge ->
                edge.getType() == BuiltinMovementType.DIG
                    && edge.getTarget().equals(new Node(1, 1, 0))),
            "digging must preserve exact collision support such as snow layers");
    }

    @Test
    void defaultContextRetainsRegisteredGlobalStrategies() {
        java.util.concurrent.atomic.AtomicInteger calls = new java.util.concurrent.atomic.AtomicInteger();
        PathfindingContextBuilder.registerGlobalStrategy((node, ignoredWorld) -> {
            calls.incrementAndGet();
            return List.of();
        });

        new DefaultPathfindingContext(world).getEdges(new Node(0, 64, 0));

        assertTrue(calls.get() > 0,
            "default contexts must preserve plugin-registered movement strategies");
    }
}
