package xin.bbtt.pathfinding;

import lombok.Getter;

/**
 * Represents a directed edge in the pathfinding graph.
 * Connects the current node to a target node with a specific cost.
 */
@Getter
public class Edge {
    private final Node target;
    private final double cost;

    public Edge(Node target, double cost) {
        this.target = target;
        this.cost = cost;
    }
}
