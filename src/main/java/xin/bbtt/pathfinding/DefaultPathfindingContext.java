package xin.bbtt.pathfinding;

import xin.bbtt.world.World;
import java.util.List;

/**
 * A standard PathfindingContext that includes all default movement strategies.
 */
public class DefaultPathfindingContext implements PathfindingContext {
    private final PathfindingContext internalContext;

    public DefaultPathfindingContext(World world) {
        this.internalContext = new PathfindingContextBuilder(world)
                .addDefaultStrategies()
                .build();
    }

    @Override
    public List<Edge> getEdges(Node u) {
        return internalContext.getEdges(u);
    }

    @Override
    public double getHeuristic(Node a, Node b) {
        return internalContext.getHeuristic(a, b);
    }
}
