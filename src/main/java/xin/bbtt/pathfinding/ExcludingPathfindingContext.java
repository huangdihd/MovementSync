package xin.bbtt.pathfinding;

import java.util.List;
import java.util.Set;

/** A request-local execution veto; it never changes the shared world or strategies. */
public final class ExcludingPathfindingContext implements PathfindingContext {
    public record Transition(Node from, Node to) {}

    private final java.util.Map<MovementType, Integer> rejectedLabels = new java.util.HashMap<>();

    /** Counts filtered edge candidates by movement label during this search. */
    public java.util.Map<MovementType, Integer> rejectedLabels() { return java.util.Map.copyOf(rejectedLabels); }

    private final PathfindingContext delegate;
    private final Set<Transition> excluded;

    public ExcludingPathfindingContext(PathfindingContext delegate, Set<Transition> excluded) {
        this.delegate = delegate;
        this.excluded = Set.copyOf(excluded);
    }

    @Override
    public List<Edge> getEdges(Node node) {
        // Another movement label for the same departure is not an alternate route.
        return delegate.getEdges(node).stream()
                .filter(edge -> {
                    if (!excluded.contains(new Transition(node, edge.getTarget()))) return true;
                    rejectedLabels.merge(edge.getType(), 1, Integer::sum);
                    xin.bbtt.MovementSync.getLogger().debug("Path excluded edge source={} target={} label={}",
                            node, edge.getTarget(), edge.getType().name());
                    return false;
                })
                .toList();
    }

    @Override
    public double getHeuristic(Node a, Node b) {
        return delegate.getHeuristic(a, b);
    }

    @Override
    public boolean isUnloadedFrontier(Node node, Node goal) {
        return delegate.isUnloadedFrontier(node, goal);
    }
}
