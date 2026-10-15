package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.*;
import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

class SearchQueryCacheTest {
    @Test
    void cachesPositiveAndNegativeQueriesOnlyInsideSearch() {
        CountingWorld world = new CountingWorld();
        try (var ignored = StandablePositionResolver.beginSearch()) {
            assertTrue(StandablePositionResolver.feetY(world, 0, 1, 0).isPresent());
            assertTrue(StandablePositionResolver.feetY(world, 0, 2, 0).isEmpty());
            int reads = world.reads;
            StandablePositionResolver.feetY(world, 0, 1, 0);
            StandablePositionResolver.feetY(world, 0, 2, 0);
            StandablePositionResolver.supportY(world, 0, 1, 0);
            assertEquals(reads, world.reads);
            CountingWorld other = new CountingWorld();
            other.solid = false;
            assertTrue(StandablePositionResolver.feetY(other, 0, 1, 0).isEmpty());
        }
        world.solid = false;
        assertTrue(StandablePositionResolver.feetY(world, 0, 1, 0).isEmpty());
    }

    @Test
    void cachedAndUncachedStrategiesProduceIdenticalOrderedEdges() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }
            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int z = (int) Math.floor(position.z);
                if (position.y < 0) return 1;
                if (position.y < 1 && Math.floorMod(x + z, 4) == 0) return 12694;
                if (position.y < 2 && Math.floorMod(x - z, 5) == 0) return 1;
                return 0;
            }
        };
        List<MovementStrategy> strategies = List.of(new WalkStrategy(true), new JumpStrategy(true),
            new FallStrategy(3), new GapJumpStrategy());
        for (int x = -2; x <= 2; x++) {
            Node source = new Node(x, 1, 0);
            for (MovementStrategy strategy : strategies) {
                List<Edge> expected = strategy.findEdges(source, world);
                try (var ignored = StandablePositionResolver.beginSearch()) {
                    for (int repeat = 0; repeat < 2; repeat++) {
                        List<Edge> actual = strategy.findEdges(source, world);
                        assertEquals(expected.size(), actual.size());
                        for (int index = 0; index < expected.size(); index++) {
                            assertEquals(expected.get(index).getTarget(), actual.get(index).getTarget());
                            assertEquals(expected.get(index).getType(), actual.get(index).getType());
                            assertEquals(expected.get(index).getCost(), actual.get(index).getCost());
                        }
                    }
                }
            }
        }
    }

    @Test
    void nestedSearchRestoresOuterCache() {
        CountingWorld world = new CountingWorld();
        try (var outer = StandablePositionResolver.beginSearch()) {
            StandablePositionResolver.feetY(world, 0, 1, 0);
            try (var inner = StandablePositionResolver.beginSearch()) {
                StandablePositionResolver.feetY(world, 0, 1, 0);
            }
            int reads = world.reads;
            StandablePositionResolver.feetY(world, 0, 1, 0);
            assertEquals(reads, world.reads);
        }
    }

    @Test
    void exceptionDoesNotLeakSearchCache() {
        CountingWorld world = new CountingWorld();
        PathfindingContext context = new PathfindingContext() {
            public List<Edge> getEdges(Node node) {
                StandablePositionResolver.feetY(world, 0, 1, 0);
                throw new IllegalStateException("test");
            }
            public double getHeuristic(Node source, Node target) { return 1; }
        };
        assertThrows(IllegalStateException.class, () ->
            new DStarLite(new Node(0, 1, 0), new Node(1, 1, 0), context).findPath(10));
        world.solid = false;
        assertTrue(StandablePositionResolver.feetY(world, 0, 1, 0).isEmpty());
    }

    @Test
    void unloadedDestinationCannotProduceDefaultMovementEdges() {
        World world = new CountingWorld() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return chunkX == 0 && chunkZ == 0; }
        };
        GenericPathfindingContext context = new GenericPathfindingContext(world, List.of(
            new WalkStrategy(true), new JumpStrategy(true), new FallStrategy(3), new GapJumpStrategy()));
        for (Edge edge : context.getEdges(new Node(15, 1, 8))) {
            assertTrue(world.chunkLoaded(edge.getTarget().x >> 4, edge.getTarget().z >> 4));
        }
        List<PathStep> path = new DStarLite(new Node(14, 1, 8), new Node(16, 1, 8), context).findPath(30);
        assertFalse(path.isEmpty());
        assertTrue(path.stream().allMatch(step -> step.getNode().x < 16));
    }

    @Test
    void boundaryReturnsDoNotQueryContextAndPreserveNegativeBudgetBehavior() {
        PathfindingContext context = new PathfindingContext() {
            public List<Edge> getEdges(Node node) { throw new AssertionError("edges"); }
            public double getHeuristic(Node source, Node target) { throw new AssertionError("heuristic"); }
        };
        Node same = new Node(0, 1, 0);
        assertEquals(1, new DStarLite(same, same, context).findPath(0).size());
        assertTrue(new DStarLite(same, same, context).findPath(-1).isEmpty());
        assertTrue(new DStarLite(same, new Node(1, 1, 0), context).findPath(-1).isEmpty());
    }

    private static class CountingWorld extends World {
        int reads;
        boolean solid = true;
        @Override
        public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }
        @Override
        public int getBlockAt(Vector3d position) {
            reads++;
            return solid && position.y < 1 ? 1 : 0;
        }
    }
}
