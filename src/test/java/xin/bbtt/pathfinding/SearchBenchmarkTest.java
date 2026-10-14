package xin.bbtt.pathfinding;

import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import xin.bbtt.world.World;

/** Run each terrain in a fresh JVM: mvn -q -o -Dtest=SearchBenchmarkTest
 * -Dsearch.benchmark=flat150 test (also flat300, uneven, jump, water).
 * First sample is cold; subsequent samples include three warmups.
 */
@EnabledIfSystemProperty(named = "search.benchmark", matches = ".+")
class SearchBenchmarkTest {
    @Test
    void benchmark() {
        String terrain = System.getProperty("search.benchmark");
        int distance = switch (terrain) {
            case "flat150" -> 150;
            case "flat300" -> 300;
            case "uneven" -> 150;
            default -> 12;
        };
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (terrain.equals("water") && x >= 3 && x < 9 && y < 1) return 86;
                if (terrain.equals("jump")) {
                    if (x >= 6 && x < 8 && y >= -3) return 0;
                    if (x >= 2 && x < 4 && y == 1) return 1;
                }
                if (terrain.equals("uneven") && x >= 16 && x < 144
                        && x % 32 < 8 && z >= -32 && z < 32 && y < 4) return 1;
                return y < 1 ? 1 : 0;
            }
        };
        for (int sample = 0; sample < 9; sample++) {
            PathfindingContext delegate = new PathfindingContextBuilder(world).addNoPlacementStrategies(false).build();
            int[] expansions = {0};
            long[] edgeTime = {0};
            PathfindingContext context = new PathfindingContext() {
                @Override public List<Edge> getEdges(Node node) {
                    expansions[0]++;
                    long started = System.nanoTime();
                    List<Edge> edges = delegate.getEdges(node);
                    edgeTime[0] += System.nanoTime() - started;
                    return edges;
                }
                @Override public double getHeuristic(Node source, Node target) {
                    return delegate.getHeuristic(source, target);
                }
            };
            long started = System.nanoTime();
            List<PathStep> path = new DStarLite(new Node(0, 1, 0), new Node(distance, 1, 0), context).findPath(5000);
            double milliseconds = (System.nanoTime() - started) / 1e6;
            double cost = 0;
            for (int index = 1; index < path.size(); index++) {
                PathStep step = path.get(index);
                cost += delegate.getEdges(path.get(index - 1).getNode()).stream()
                        .filter(edge -> edge.getTarget().equals(step.getNode()) && edge.getType().equals(step.getType()))
                        .mapToDouble(Edge::getCost).min().orElseThrow();
            }
            double expectedCost = switch (terrain) {
                case "flat150" -> 150;
                case "flat300" -> 300;
                case "uneven" -> 150.195959492893;
                case "jump" -> 16.328427124746;
                case "water" -> 19;
                default -> throw new IllegalArgumentException(terrain);
            };
            int expectedNodes = switch (terrain) {
                case "flat150" -> 151;
                case "flat300" -> 301;
                case "uneven" -> 128;
                case "jump" -> 11;
                default -> 13;
            };
            org.junit.jupiter.api.Assertions.assertEquals(expectedCost, cost, 1e-10);
            org.junit.jupiter.api.Assertions.assertEquals(expectedNodes, path.size());
            org.junit.jupiter.api.Assertions.assertEquals(terrain.equals("uneven")
                    ? new Node(127, 1, 8) : new Node(distance, 1, 0), path.get(path.size() - 1).getNode());
            System.out.printf("BENCH %s sample=%d ms=%.3f expanded=%d us/node=%.3f edges_ms=%.3f nodes=%d cost=%.12f end=%s%n",
                    terrain, sample, milliseconds, expansions[0], milliseconds * 1000 / expansions[0],
                    edgeTime[0] / 1e6, path.size(), cost, path.get(path.size() - 1).getNode());
        }
    }
}
