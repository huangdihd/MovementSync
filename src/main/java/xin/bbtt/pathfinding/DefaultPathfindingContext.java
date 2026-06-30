package xin.bbtt.pathfinding;

import xin.bbtt.world.World;
import java.util.List;

/**
 * A standard PathfindingContext that includes all default movement strategies.
 */
public class DefaultPathfindingContext implements PathfindingContext {
    private final PathfindingContext internalContext;

    public DefaultPathfindingContext(World world) {
        // Decide once at planning time: without blocks in the inventory,
        // placement (bridge/pillar) edges are unusable anyway.
        boolean hasBlocks = xin.bbtt.MovementSync.INSTANCE != null
                && xin.bbtt.MovementSync.INSTANCE.getInventoryManager().findBlock() != -1;
        PathfindingContextBuilder builder = new PathfindingContextBuilder(world);
        this.internalContext = (hasBlocks
                ? builder.addDefaultStrategies()
                : builder.addNoPlacementStrategies())
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
