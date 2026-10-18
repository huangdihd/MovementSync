package xin.bbtt.pathfinding;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

final class ExcludedRouteSearchDiagnosticsTest {
    private static final Node START = new Node(0, 0, 0);
    private static final Node GOAL = new Node(2, 0, 0);

    private static PathfindingContext graph(boolean unloaded) {
        return new PathfindingContext() {
            @Override public List<Edge> getEdges(Node node) {
                if (node.equals(START)) return List.of(
                        new Edge(GOAL, 1, BuiltinMovementType.WALK),
                        new Edge(GOAL, 1, BuiltinMovementType.GAP_JUMP),
                        new Edge(new Node(-1, 0, 0), 1, BuiltinMovementType.WALK));
                // The only alternative first travels 32 blocks AWAY from the goal.
                if (node.x < 0 && node.x > -32) return List.of(
                        new Edge(new Node(node.x - 1, 0, 0), 1, BuiltinMovementType.WALK));
                if (node.x == -32) return List.of(new Edge(GOAL, 34, BuiltinMovementType.WALK));
                return List.of();
            }
            @Override public double getHeuristic(Node a, Node b) { return Math.abs(a.x - b.x); }
            @Override public boolean isUnloadedFrontier(Node node, Node goal) {
                return unloaded && node.equals(START);
            }
        };
    }

    @Test void excludedDepartureRunsFullSearchEvenWhenEveryDetourNodeIsFartherAway() {
        var context = new ExcludingPathfindingContext(graph(false),
                Set.of(new ExcludingPathfindingContext.Transition(START, GOAL)));
        var search = new DStarLite(START, GOAL, context);
        var path = search.findPath(100);
        assertEquals(34, path.size());
        assertEquals(new Node(-32, 0, 0), path.get(32).getNode());
        assertEquals(GOAL, path.get(33).getNode());
        assertEquals(DStarLite.Termination.COMPLETE, search.diagnostics().termination());
        assertEquals(33, search.diagnostics().expandedNodes()); // Goal recognition does not expand edges.
        assertEquals(GOAL, search.diagnostics().endpoint());
        assertEquals(1, context.rejectedLabels().get(BuiltinMovementType.WALK));
        assertEquals(1, context.rejectedLabels().get(BuiltinMovementType.GAP_JUMP));
    }

    @Test void budgetIsExactAndDoesNotClaimAnExhaustiveFailureOrUnloadedFrontier() {
        var context = new ExcludingPathfindingContext(graph(false),
                Set.of(new ExcludingPathfindingContext.Transition(START, GOAL)));
        var search = new DStarLite(START, GOAL, context);
        assertTrue(search.findPath(0).isEmpty());
        assertEquals(0, search.diagnostics().expandedNodes());
        assertEquals(DStarLite.Termination.BUDGET_EXHAUSTED, search.diagnostics().termination());
        assertTrue(search.findPath(5).isEmpty(), "no explored node is closer; a detour still exists");
        assertEquals(5, search.diagnostics().expandedNodes());
        assertEquals(1, search.diagnostics().openNodes());
        assertEquals(DStarLite.Termination.BUDGET_EXHAUSTED, search.diagnostics().termination());
        assertEquals(34, search.findPath(34).size());
        assertEquals(DStarLite.Termination.COMPLETE, search.diagnostics().termination());
    }

    @Test void unloadedFrontierAndExhaustedGraphHaveDifferentDiagnostics() {
        var search = new DStarLite(START, GOAL, graph(true));
        assertEquals(1, search.findPath(100).size());
        assertEquals(DStarLite.Termination.UNLOADED_FRONTIER, search.diagnostics().termination());
        assertEquals(1, search.diagnostics().expandedNodes());
        assertEquals(START, search.diagnostics().endpoint());

        search = new DStarLite(new Node(99, 0, 0), GOAL, graph(false));
        assertTrue(search.findPath(100).isEmpty());
        assertEquals(DStarLite.Termination.EXHAUSTED, search.diagnostics().termination());
        assertEquals(0, search.diagnostics().openNodes());
        assertEquals(1, search.diagnostics().expandedNodes());
    }
}
