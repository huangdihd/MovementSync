package xin.bbtt.pathfinding;

import java.util.List;

/** Bounded retry for a search that has not found even a usable departure. */
public final class WalkingSearch {
    private WalkingSearch() {}

    public static List<PathStep> findPath(DStarLite search) {
        List<PathStep> path = search.findPath(2000);
        // A partial route is already useful. Unknown terrain and an exhausted
        // graph cannot be repaired by spending more nodes. Do not reset the
        // execution watchdog: additional search work is not physical progress.
        for (int budget = 4000; budget <= 8000 && path.size() <= 1
                && search.diagnostics().termination() == DStarLite.Termination.BUDGET_EXHAUSTED;
                budget *= 2) {
            path = search.findPath(budget);
        }
        return path;
    }
}
