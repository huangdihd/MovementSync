package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

/**
 * Deterministic flat-world benchmark: three warmups, five measured searches.
 * Java 17/macOS arm64 local medians: before 73.467 ms, after 51.744 ms;
 * both expand 150 nodes and return 151 WALK nodes. Timing is observational,
 * not a CI threshold; the report is written to target/pathfinding-150-benchmark.txt.
 */
class PathfindingPerformanceTest {
    @Test
    void unloadedGoalsStopAtFrontierWellBeforeIterationLimit() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return chunkX >= -1 && chunkX <= 1 && chunkZ >= -1 && chunkZ <= 1;
            }

            @Override
            public int getBlockAt(Vector3d position) { return position.y < 1 ? 1 : 0; }
        };
        for (int distance : new int[] {150, 600, -150, -600}) {
            int[] iterations = {0};
            int[] expansions = {0};
            PathfindingContext context = new GenericPathfindingContext(world, List.of(
                    new WalkStrategy(), new JumpStrategy(), new FallStrategy(3), new GapJumpStrategy())) {
                @Override
                public boolean isUnloadedFrontier(Node node, Node goal) {
                    iterations[0]++;
                    return super.isUnloadedFrontier(node, goal);
                }

                @Override
                public List<Edge> getEdges(Node node) {
                    expansions[0]++;
                    return super.getEdges(node);
                }
            };
            List<PathStep> path = new DStarLite(new Node(0, 1, 0), new Node(distance, 1, 0), context)
                .findPath(5000);
            int boundary = distance > 0 ? 31 : -16;
            assertEquals(new Node(boundary, 1, 0), path.get(path.size() - 1).getNode());
            assertEquals(Math.abs(boundary), expansions[0]);
            assertEquals(expansions[0] + 1, iterations[0]);
            assertTrue(iterations[0] < 100, "must stop early, not exhaust the 5000-iteration budget");
            for (PathStep step : path) {
                Node node = step.getNode();
                assertTrue(world.chunkLoaded(node.x >> 4, node.z >> 4));
            }
        }
    }

    @Test
    void frontierRequiresUnloadedGoalAndBoundaryTowardIt() {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return chunkX == 0 && chunkZ == 0;
            }
        };
        PathfindingContext context = new GenericPathfindingContext(world, List.of());
        assertTrue(context.isUnloadedFrontier(new Node(15, 1, 8), new Node(600, 1, 8)));
        assertFalse(context.isUnloadedFrontier(new Node(0, 1, 8), new Node(600, 1, 8)));
        assertFalse(context.isUnloadedFrontier(new Node(8, 1, 0), new Node(600, 1, 8)));
        assertFalse(context.isUnloadedFrontier(new Node(15, 1, 8), new Node(8, 1, 8)));
        assertFalse(context.isUnloadedFrontier(new Node(16, 1, 8), new Node(600, 1, 8)));
        assertTrue(new DefaultPathfindingContext(world)
            .isUnloadedFrontier(new Node(15, 1, 8), new Node(600, 1, 8)));
    }

    @Test
    void flat150BlockSearchBenchmark() throws java.io.IOException {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }

            @Override
            public int getBlockAt(Vector3d position) { return position.y < 1 ? 1 : 0; }
        };
        List<Double> milliseconds = new ArrayList<>();
        for (int sample = 0; sample < 8; sample++) {
            int[] iterations = {0};
            GenericPathfindingContext delegate = new GenericPathfindingContext(world, List.of(
                new WalkStrategy(), new JumpStrategy(), new FallStrategy(3), new GapJumpStrategy()));
            PathfindingContext context = new PathfindingContext() {
                @Override
                public List<Edge> getEdges(Node node) {
                    iterations[0]++;
                    return delegate.getEdges(node);
                }

                @Override
                public double getHeuristic(Node source, Node target) {
                    return delegate.getHeuristic(source, target);
                }
            };
            long started = System.nanoTime();
            List<PathStep> path = new DStarLite(new Node(0, 1, 0), new Node(150, 1, 0), context)
                .findPath(1000);
            double elapsed = (System.nanoTime() - started) / 1_000_000.0;
            assertEquals(151, path.size());
            for (PathStep step : path) assertEquals(BuiltinMovementType.WALK, step.getType());
            assertEquals(new Node(150, 1, 0), path.get(path.size() - 1).getNode());
            assertEquals(150, iterations[0]);
            if (sample >= 3) milliseconds.add(elapsed);
        }
        Collections.sort(milliseconds);
        String result = String.format("150-block search: 150 expansions, median %.3f ms, %.3f ms/expansion, samples=%s%n",
            milliseconds.get(2), milliseconds.get(2) / 150, milliseconds);
        java.nio.file.Files.writeString(java.nio.file.Path.of("target/pathfinding-150-benchmark.txt"), result);
    }
}
