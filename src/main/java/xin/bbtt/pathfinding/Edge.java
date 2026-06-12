package xin.bbtt.pathfinding;

import lombok.Getter;

/**
 * Represents a directed edge in the pathfinding graph.
 * Connects the current node to a target node with a specific cost and the
 * movement type required to traverse it.
 */
@Getter
public class Edge {
    private final Node target;
    private final double cost;
    private final MovementType type;

    /** Strategies that don't declare a type default to plain walking. */
    public Edge(Node target, double cost) {
        this(target, cost, BuiltinMovementType.WALK);
    }

    public Edge(Node target, double cost, MovementType type) {
        this.target = target;
        this.cost = cost;
        this.type = type;
    }
}
