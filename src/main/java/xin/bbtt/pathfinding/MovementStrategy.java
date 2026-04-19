package xin.bbtt.pathfinding;

import xin.bbtt.world.World;
import java.util.List;

public interface MovementStrategy {
    /**
     * Finds and returns all reachable outgoing edges from the current node u using this strategy.
     * Each edge contains the target node and the cost of the movement.
     */
    List<Edge> findEdges(Node u, World world);
}
