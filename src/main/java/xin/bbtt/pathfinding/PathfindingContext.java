package xin.bbtt.pathfinding;

import java.util.List;

public interface PathfindingContext {
    /**
     * Returns a list of outgoing edges from the given node.
     */
    List<Edge> getEdges(Node node);

    /**
     * Returns a heuristic estimate of the cost from node 'a' to the goal 'b'.
     */
    double getHeuristic(Node a, Node b);
}
