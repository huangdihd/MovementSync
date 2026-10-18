package xin.bbtt.pathfinding;

import java.util.List;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class WalkingSearchBudgetTest {
    private static final Node START = new Node(172, 40, -31);
    private static final Node GOAL = new Node(175, 60, -32);

    private PathfindingContext detour(int length) {
        return new PathfindingContext() {
            public List<Edge> getEdges(Node n) {
                if (n.equals(START)) return List.of(new Edge(new Node(171, 40, -31), 1, BuiltinMovementType.WALK));
                if (n.x > 172 - length) return List.of(new Edge(new Node(n.x - 1, 40, -31), 1, BuiltinMovementType.WALK));
                if (n.x == 172 - length) return List.of(new Edge(GOAL, length + 20, BuiltinMovementType.WALK));
                return List.of();
            }
            public double getHeuristic(Node a, Node b) {
                return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2) + Math.pow(a.z - b.z, 2));
            }
        };
    }

    @Test void stalledVerticalGoalGetsBoundedRetryWithoutClaimingNoRoute() {
        var search = new DStarLite(START, GOAL, detour(2100));
        assertTrue(search.findPath(2000).isEmpty());
        assertEquals(DStarLite.Termination.BUDGET_EXHAUSTED, search.diagnostics().termination());
        var path = WalkingSearch.findPath(search);
        assertEquals(GOAL, path.get(path.size() - 1).getNode());
        assertEquals(4000, search.diagnostics().budget());
        assertEquals(DStarLite.Termination.COMPLETE, search.diagnostics().termination());
    }

    @Test void retryHasHardCapAndDoesNotInventProgress() {
        var search = new DStarLite(START, GOAL, detour(9000));
        assertTrue(WalkingSearch.findPath(search).isEmpty());
        assertEquals(8000, search.diagnostics().budget());
        assertEquals(8000, search.diagnostics().expandedNodes());
        assertEquals(DStarLite.Termination.BUDGET_EXHAUSTED, search.diagnostics().termination());
    }
}
