package xin.bbtt.pathfinding;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Stub PathfindingContext for testing DStarLite on a simple flat (y=0) 2D grid.
 * Each node connects to its 4 cardinal neighbours (N/S/E/W) at cost 1.0.
 * Cells in {@code blocked} produce no outgoing edges.
 */
public class GridContext implements PathfindingContext {

    private final Set<Node> blocked;

    public GridContext(Set<Node> blocked) {
        this.blocked = blocked;
    }

    @Override
    public List<Edge> getEdges(Node node) {
        List<Edge> edges = new ArrayList<>();
        if (blocked.contains(node)) return edges;
        Node[] neighbours = {
            new Node(node.x + 1, 0, node.z),
            new Node(node.x - 1, 0, node.z),
            new Node(node.x, 0, node.z + 1),
            new Node(node.x, 0, node.z - 1),
        };
        for (Node n : neighbours) {
            if (!blocked.contains(n)) {
                edges.add(new Edge(n, 1.0));
            }
        }
        return edges;
    }

    @Override
    public double getHeuristic(Node a, Node b) {
        // Euclidean distance on the XZ plane (all nodes at y=0).
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.z - b.z, 2));
    }
}
