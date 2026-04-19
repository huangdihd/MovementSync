package xin.bbtt.pathfinding;

import xin.bbtt.world.World;
import java.util.ArrayList;
import java.util.List;

/**
 * A flexible PathfindingContext that uses a dynamic list of strategies.
 */
public class GenericPathfindingContext implements PathfindingContext {
    private final World world;
    private final List<MovementStrategy> strategies;

    public GenericPathfindingContext(World world, List<MovementStrategy> strategies) {
        this.world = world;
        this.strategies = strategies;
    }

    @Override
    public List<Edge> getEdges(Node u) {
        List<Edge> allEdges = new ArrayList<>();
        for (MovementStrategy strategy : strategies) {
            allEdges.addAll(strategy.findEdges(u, world));
        }
        return allEdges;
    }

    @Override
    public double getHeuristic(Node a, Node b) {
        // Standard Euclidean distance as heuristic
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2) + Math.pow(a.z - b.z, 2));
    }
}
