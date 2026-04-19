package xin.bbtt.pathfinding;

import xin.bbtt.world.World;
import java.util.ArrayList;
import java.util.List;

/**
 * Fluent builder for creating PathfindingContext instances.
 */
public class PathfindingContextBuilder {
    private final World world;
    private final List<MovementStrategy> strategies = new ArrayList<>();
    private static final List<MovementStrategy> globalStrategies = new ArrayList<>();

    public PathfindingContextBuilder(World world) {
        this.world = world;
    }

    public static void registerGlobalStrategy(MovementStrategy strategy) {
        globalStrategies.add(strategy);
    }

    public PathfindingContextBuilder addStrategy(MovementStrategy strategy) {
        strategies.add(strategy);
        return this;
    }

    public PathfindingContextBuilder addWalk() {
        return addStrategy(new WalkStrategy());
    }

    public PathfindingContextBuilder addJump() {
        return addStrategy(new JumpStrategy());
    }

    public PathfindingContextBuilder addFall() {
        return addStrategy(new FallStrategy(3)); // Default fall height 3
    }

    public PathfindingContextBuilder addGapJump() {
        return addStrategy(new GapJumpStrategy());
    }

    public PathfindingContextBuilder addBridgePillar() {
        return addStrategy(new BridgePillarStrategy());
    }

    /**
     * Adds all standard movement strategies at once, including global ones.
     */
    public PathfindingContextBuilder addDefaultStrategies() {
        this.addWalk()
            .addJump()
            .addFall()
            .addGapJump()
            .addBridgePillar();
        
        for (MovementStrategy strategy : globalStrategies) {
            this.addStrategy(strategy);
        }
        return this;
    }

    public PathfindingContext build() {
        return new GenericPathfindingContext(world, new ArrayList<>(strategies));
    }
}
