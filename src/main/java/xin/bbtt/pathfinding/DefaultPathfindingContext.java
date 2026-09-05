package xin.bbtt.pathfinding;

import xin.bbtt.world.World;
import java.util.List;

/**
 * A standard PathfindingContext that includes all default movement strategies.
 */
public class DefaultPathfindingContext implements PathfindingContext {
    private final PathfindingContext internalContext;

    public DefaultPathfindingContext(World world) {
        this(world, false);
    }

    /**
     * @param allowDigging when false, the planner only routes across standable
     *     terrain and never mines obstacles; doors, walls and terrain stay intact.
     */
    public DefaultPathfindingContext(World world, boolean allowDigging) {
        // Decide once at planning time: without blocks in the inventory,
        // placement (bridge/pillar) edges are unusable anyway.
        boolean hasBlocks = xin.bbtt.MovementSync.INSTANCE != null
                && xin.bbtt.MovementSync.INSTANCE.getInventoryManager().findBlock() != -1;
        PathfindingContextBuilder builder = new PathfindingContextBuilder(world);
        this.internalContext = (hasBlocks
                ? builder.addDefaultStrategies(allowDigging)
                : builder.addNoPlacementStrategies(allowDigging))
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
