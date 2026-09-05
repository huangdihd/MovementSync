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

    /** Walk strategy with explicit control over dig-through-obstacle edges. */
    public PathfindingContextBuilder addWalk(boolean allowDigging) {
        return addStrategy(new WalkStrategy(allowDigging));
    }

    public PathfindingContextBuilder addJump() {
        return addStrategy(new JumpStrategy());
    }

    /** Jump strategy with explicit control over dig-the-target-cell edges. */
    public PathfindingContextBuilder addJump(boolean allowDigging) {
        return addStrategy(new JumpStrategy(allowDigging));
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
        return addDefaultStrategies(false);
    }

    public PathfindingContextBuilder addDefaultStrategies(boolean allowDigging) {
        this.addWalk(allowDigging)
            .addJump(allowDigging)
            .addFall()
            .addGapJump()
            .addBridgePillar();

        for (MovementStrategy strategy : globalStrategies) {
            this.addStrategy(strategy);
        }
        return this;
    }

    /**
     * Adds the standard movement strategies except block placement
     * (bridging/pillaring), including global ones.
     */
    public PathfindingContextBuilder addNoPlacementStrategies() {
        return addNoPlacementStrategies(false);
    }

    public PathfindingContextBuilder addNoPlacementStrategies(boolean allowDigging) {
        this.addWalk(allowDigging)
            .addJump(allowDigging)
            .addFall()
            .addGapJump();

        for (MovementStrategy strategy : globalStrategies) {
            this.addStrategy(strategy);
        }
        return this;
    }

    public PathfindingContext build() {
        return new GenericPathfindingContext(world, new ArrayList<>(strategies));
    }
}
