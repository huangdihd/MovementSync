package xin.bbtt.pathfinding;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

final class ExcludingPathfindingContextTest {
    @Test void rejectionSelectsDetourWithoutPoisoningAnotherRequestOrReverseEdge() {
        Node start = new Node(27, 33, -1), goal = new Node(30, 33, -1);
        Node detour = new Node(27, 33, -2);
        PathfindingContext graph = new PathfindingContext() {
            @Override public List<Edge> getEdges(Node node) {
                if (node.equals(start)) return List.of(
                        new Edge(goal, 1, BuiltinMovementType.GAP_JUMP),
                        new Edge(goal, 2, BuiltinMovementType.WALK),
                        new Edge(detour, 3, BuiltinMovementType.WALK));
                if (node.equals(detour)) return List.of(new Edge(goal, 3, BuiltinMovementType.WALK));
                if (node.equals(goal)) return List.of(new Edge(start, 1, BuiltinMovementType.GAP_JUMP));
                return List.of();
            }
            @Override public double getHeuristic(Node a, Node b) { return 0; }
            @Override public boolean isUnloadedFrontier(Node node, Node target) { return false; }
        };
        var excluded = new HashSet<>(Set.of(new ExcludingPathfindingContext.Transition(start, goal)));
        var context = new ExcludingPathfindingContext(graph, excluded);
        excluded.clear(); // A synchronous search owns an immutable snapshot of its request's vetoes.
        var route = new DStarLite(start, goal, context).findPath(100);
        assertEquals(List.of(start, detour, goal), route.stream().map(PathStep::getNode).toList());
        assertEquals(List.of(start, goal), new DStarLite(start, goal,
                new ExcludingPathfindingContext(graph, Set.of())).findPath(100)
                .stream().map(PathStep::getNode).toList());
        assertEquals(start, context.getEdges(goal).get(0).getTarget());
        assertEquals(0, context.getHeuristic(start, goal));
    }

    @Test void unloadedFrontierSemanticsArePreserved() {
        Node node = new Node(0, 0, 0);
        PathfindingContext graph = new PathfindingContext() {
            @Override public List<Edge> getEdges(Node from) { return List.of(); }
            @Override public double getHeuristic(Node a, Node b) { return 42; }
            @Override public boolean isUnloadedFrontier(Node from, Node goal) { return true; }
        };
        var context = new ExcludingPathfindingContext(graph, Set.of());
        assertTrue(context.isUnloadedFrontier(node, node));
        assertEquals(42, context.getHeuristic(node, node));
    }
}
