package xin.bbtt.pathfinding;

import lombok.Getter;

/**
 * One step of a computed path: the node to reach and the movement type of the
 * edge that leads to it from the previous step. The first step of a path has
 * no incoming edge and uses {@link BuiltinMovementType#WALK} as a neutral
 * default.
 */
@Getter
public class PathStep {
    private final Node node;
    private final MovementType type;

    public PathStep(Node node, MovementType type) {
        this.node = node;
        this.type = type;
    }

    @Override
    public String toString() {
        return type.name() + " -> " + node;
    }
}
