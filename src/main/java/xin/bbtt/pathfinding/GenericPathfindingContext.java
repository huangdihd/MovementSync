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
    public boolean isUnloadedFrontier(Node node, Node goal) {
        if (world.chunkLoaded(goal.x >> 4, goal.z >> 4)) return false;
        if (!world.chunkLoaded(node.x >> 4, node.z >> 4)) return false;

        // Only stop at a loaded cell whose next horizontal step toward the
        // target crosses into unknown terrain, never at a boundary behind it.
        int stepX = Integer.compare(goal.x, node.x);
        int stepZ = Integer.compare(goal.z, node.z);
        return (stepX != 0 && !world.chunkLoaded((node.x + stepX) >> 4, node.z >> 4))
            || (stepZ != 0 && !world.chunkLoaded(node.x >> 4, (node.z + stepZ) >> 4));
    }

    @Override
    public double getHeuristic(Node a, Node b) {
        // Standard Euclidean distance as heuristic
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2) + Math.pow(a.z - b.z, 2));
    }
}
