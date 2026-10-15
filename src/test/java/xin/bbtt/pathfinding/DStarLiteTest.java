package xin.bbtt.pathfinding;

import static org.junit.jupiter.api.Assertions.*;

import java.util.*;

import org.junit.jupiter.api.Test;

class DStarLiteTest {

    @Test
    void customContextDoesNotOptIntoUnloadedFrontierStopping() {
        GridContext context = new GridContext(Set.of());
        Node goal = new Node(150, 0, 0);
        assertFalse(context.isUnloadedFrontier(new Node(15, 0, 0), goal));
        List<PathStep> path = new DStarLite(new Node(0, 0, 0), goal, context).findPath(5000);
        assertEquals(goal, path.get(path.size() - 1).getNode());
    }

    @Test
    void boundaryDoesNotStopSearchWhenAnOpenNodeIsCloserToGoal() {
        Node start = new Node(0, 0, 0);
        Node boundary = new Node(1, 0, 0);
        Node closer = new Node(2, 0, 0);
        Node goal = new Node(3, 0, 0);
        PathfindingContext context = new PathfindingContext() {
            @Override
            public List<Edge> getEdges(Node node) {
                if (node.equals(start)) return List.of(
                    new Edge(boundary, 1, BuiltinMovementType.WALK),
                    new Edge(closer, 10, BuiltinMovementType.WALK));
                if (node.equals(closer)) return List.of(new Edge(goal, 1, BuiltinMovementType.WALK));
                return List.of();
            }

            @Override
            public double getHeuristic(Node source, Node target) {
                return Math.abs(target.x - source.x);
            }

            @Override
            public boolean isUnloadedFrontier(Node node, Node target) {
                return node.equals(boundary);
            }
        };
        List<PathStep> path = new DStarLite(start, goal, context).findPath(100);
        assertEquals(goal, path.get(path.size() - 1).getNode());
    }

    @Test
    void staleQueueEntriesDoNotConsumeIterationBudget() {
        Node start = new Node(0, 0, 0);
        Node via = new Node(1, 0, 0);
        Node improved = new Node(2, 0, 0);
        Node goal = new Node(3, 0, 0);
        List<Node> expanded = new ArrayList<>();
        PathfindingContext context = new PathfindingContext() {
            @Override public List<Edge> getEdges(Node node) {
                expanded.add(node);
                if (node.equals(start)) return List.of(
                        new Edge(improved, 4, BuiltinMovementType.WALK),
                        new Edge(via, 1, BuiltinMovementType.WALK));
                if (node.equals(via)) return List.of(new Edge(improved, 1, BuiltinMovementType.WALK));
                if (node.equals(improved)) return List.of(new Edge(goal, 5, BuiltinMovementType.WALK));
                return List.of();
            }
            @Override public double getHeuristic(Node source, Node target) { return 0; }
        };
        List<PathStep> path = new DStarLite(start, goal, context).findPath(3);
        assertEquals(List.of(start, via, improved, goal), path.stream().map(PathStep::getNode).toList());
        assertEquals(List.of(start, via, improved), expanded);
    }

    @Test
    void straightLineOnEmptyPlane() {
        Node start = new Node(0, 0, 0);
        Node goal = new Node(5, 0, 0);
        GridContext ctx = new GridContext(Set.of());

        List<PathStep> path = new DStarLite(start, goal, ctx).findPath(200);
        assertFalse(path.isEmpty(), "path should be found on clear terrain");
        assertEquals(start, path.get(0).getNode());
        assertEquals(goal, path.get(path.size() - 1).getNode());
        assertTrue(path.size() >= 6, "path should be at least 6 nodes");
    }

    @Test
    void unreachableGoalReturnsEmpty() {
        Node start = new Node(0, 0, 0);
        Node goal = new Node(5, 0, 0);
        // Block all neighbours of start
        GridContext ctx = new GridContext(Set.of(
                new Node(1, 0, 0),
                new Node(-1, 0, 0),
                new Node(0, 0, 1),
                new Node(0, 0, -1)
        ));

        List<PathStep> path = new DStarLite(start, goal, ctx).findPath(200);
        assertTrue(path.isEmpty(), "no path when all exits are blocked from start");
    }

    @Test
    void detoursAroundObstacle() {
        Node start = new Node(0, 0, 0);
        Node goal = new Node(2, 0, 0);
        // Block the direct neighbour (1,0,0) so the path must go around.
        GridContext ctx = new GridContext(Set.of(new Node(1, 0, 0)));

        List<PathStep> path = new DStarLite(start, goal, ctx).findPath(200);
        assertFalse(path.isEmpty());
        assertEquals(start, path.get(0).getNode());
        assertEquals(goal, path.get(path.size() - 1).getNode());
    }

    @Test
    void iterationLimitReturnsPartial() {
        Node start = new Node(0, 0, 0);
        Node goal = new Node(100, 0, 0);
        GridContext ctx = new GridContext(Set.of());

        List<PathStep> path = new DStarLite(start, goal, ctx).findPath(10);
        // Should stop early due to iteration cap; may return partial.
        // The important assertion is it doesn't loop forever.
        assertNotNull(path);
        assertTrue(path.size() < 100, "should stop early — path shorter than direct line");
    }

    @Test
    void startEqualsGoalReturnsSingleNodePath() {
        Node same = new Node(3, 0, 7);
        GridContext ctx = new GridContext(Set.of());
        List<PathStep> path = new DStarLite(same, same, ctx).findPath(200);
        assertFalse(path.isEmpty());
        assertEquals(1, path.size());
        assertEquals(same, path.get(0).getNode());
    }

    @Test
    void edgeTypesArePropagatedToPathSteps() {
        Node start = new Node(0, 0, 0);
        Node mid = new Node(1, 0, 0);
        Node goal = new Node(2, 0, 0);
        // A linear context whose two edges carry distinct movement types.
        PathfindingContext ctx = new PathfindingContext() {
            @Override
            public List<Edge> getEdges(Node node) {
                if (node.equals(start)) return List.of(new Edge(mid, 1.0, BuiltinMovementType.JUMP));
                if (node.equals(mid)) return List.of(new Edge(goal, 1.0, BuiltinMovementType.GAP_JUMP));
                return List.of();
            }

            @Override
            public double getHeuristic(Node a, Node b) {
                return 0;
            }
        };

        List<PathStep> path = new DStarLite(start, goal, ctx).findPath(50);
        assertEquals(3, path.size());
        // The first step has no incoming edge and defaults to WALK.
        assertEquals(BuiltinMovementType.WALK, path.get(0).getType());
        assertEquals(BuiltinMovementType.JUMP, path.get(1).getType());
        assertEquals(BuiltinMovementType.GAP_JUMP, path.get(2).getType());
    }

    @Test
    void untypedEdgesDefaultToWalk() {
        GridContext ctx = new GridContext(Set.of());
        List<PathStep> path = new DStarLite(new Node(0, 0, 0), new Node(2, 0, 0), ctx).findPath(200);
        assertFalse(path.isEmpty());
        for (PathStep step : path) {
            assertEquals(BuiltinMovementType.WALK, step.getType());
        }
    }
}
