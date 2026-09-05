package xin.bbtt.pathfinding;

import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3i;
import xin.bbtt.world.World;

/** Resolves an integer planner node to an exact collision-supported player feet height. */
public final class StandablePositionResolver {
    public static final double PLAYER_HALF_WIDTH = 0.299;
    public static final double PLAYER_HEIGHT = 1.8;
    public static final double MAX_STEP_RISE = 0.6;
    public static final double MAX_JUMP_RISE = 1.20;
    private static final double EPSILON = 1.0e-7;

    private StandablePositionResolver() {}

    public static int nodeY(double feetY) {
        if (!Double.isFinite(feetY)) throw new IllegalArgumentException("feetY must be finite");
        return (int) Math.ceil(feetY - 1.0e-6);
    }

    public static OptionalDouble feetY(World world, int x, int nodeY, int z) {
        OptionalDouble support = supportY(world, x, nodeY, z);
        if (support.isEmpty()) return OptionalDouble.empty();
        double feetY = support.getAsDouble();
        double centerX = x + 0.5;
        double centerZ = z + 0.5;
        boolean blocked = world.isBoxColliding(
            centerX - PLAYER_HALF_WIDTH,
            feetY + EPSILON,
            centerZ - PLAYER_HALF_WIDTH,
            centerX + PLAYER_HALF_WIDTH,
            feetY + PLAYER_HEIGHT,
            centerZ + PLAYER_HALF_WIDTH
        );
        return blocked ? OptionalDouble.empty() : OptionalDouble.of(feetY);
    }

    public static OptionalDouble supportY(World world, int x, int nodeY, int z) {
        if (!world.chunkLoaded(x >> 4, z >> 4)) return OptionalDouble.empty();
        double centerX = x + 0.5;
        double centerZ = z + 0.5;
        return world.findHighestCollisionTop(
            centerX - PLAYER_HALF_WIDTH,
            nodeY - 1.0 + EPSILON,
            centerZ - PLAYER_HALF_WIDTH,
            centerX + PLAYER_HALF_WIDTH,
            nodeY + EPSILON,
            centerZ + PLAYER_HALF_WIDTH
        );
    }

    public static boolean canWalk(World world, Node source, Node target) {
        OptionalDouble rise = rise(world, source, target);
        return rise.isPresent() && rise.getAsDouble() <= MAX_STEP_RISE + EPSILON;
    }

    public static boolean canJump(World world, Node source, Node target) {
        OptionalDouble sourceFeetY = expectedSourceFeetY(world, source);
        OptionalDouble targetFeetY = feetY(world, target.x, target.y, target.z);
        if (sourceFeetY.isEmpty() || targetFeetY.isEmpty()) return false;
        double rise = targetFeetY.getAsDouble() - sourceFeetY.getAsDouble();
        return rise <= MAX_JUMP_RISE + EPSILON
            && hasJumpClearance(world, source)
            && hasDestinationJumpClearance(
                world, target, sourceFeetY.getAsDouble(), targetFeetY.getAsDouble());
    }

    public static boolean canFall(World world, Node source, Node target, double maximumDrop) {
        OptionalDouble rise = rise(world, source, target);
        if (rise.isEmpty()) return false;
        double drop = -rise.getAsDouble();
        return drop > EPSILON && drop <= maximumDrop + EPSILON;
    }

    public static boolean canStepToFeetY(World world, Node source, double targetFeetY) {
        return canRiseToFeetY(world, source, targetFeetY, MAX_STEP_RISE);
    }

    public static boolean canJumpToFeetY(World world, Node source, double targetFeetY) {
        return canRiseToFeetY(world, source, targetFeetY, MAX_JUMP_RISE)
            && hasJumpClearance(world, source);
    }

    public static boolean canWalkToSupport(World world, Node source, Node target) {
        return canReachSupport(world, source, target, MAX_STEP_RISE);
    }

    public static boolean canJumpToSupport(World world, Node source, Node target) {
        return canReachSupport(world, source, target, MAX_JUMP_RISE)
            && hasJumpClearance(world, source);
    }

    private static boolean canRiseToFeetY(
            World world, Node source, double targetFeetY, double maximumRise) {
        OptionalDouble sourceFeetY = expectedSourceFeetY(world, source);
        return sourceFeetY.isPresent()
            && targetFeetY - sourceFeetY.getAsDouble() <= maximumRise + EPSILON;
    }

    private static OptionalDouble rise(World world, Node source, Node target) {
        OptionalDouble sourceFeetY = expectedSourceFeetY(world, source);
        OptionalDouble targetFeetY = feetY(world, target.x, target.y, target.z);
        if (sourceFeetY.isEmpty() || targetFeetY.isEmpty()) return OptionalDouble.empty();
        return OptionalDouble.of(targetFeetY.getAsDouble() - sourceFeetY.getAsDouble());
    }

    private static boolean canReachSupport(
            World world, Node source, Node target, double maximumRise) {
        OptionalDouble sourceFeetY = expectedSourceFeetY(world, source);
        OptionalDouble targetSupportY = supportY(world, target.x, target.y, target.z);
        return sourceFeetY.isPresent() && targetSupportY.isPresent()
            && targetSupportY.getAsDouble() - sourceFeetY.getAsDouble()
                <= maximumRise + EPSILON;
    }

    private static boolean hasJumpClearance(World world, Node source) {
        OptionalDouble actualFeetY = feetY(world, source.x, source.y, source.z);
        OptionalDouble sourceFeetY = actualFeetY.isPresent()
            ? actualFeetY
            : expectedSourceFeetY(world, source);
        if (sourceFeetY.isEmpty()) return false;
        double centerX = source.x + 0.5;
        double centerZ = source.z + 0.5;
        double feetY = sourceFeetY.getAsDouble();
        // A DIG edge clears the source's feet/head cells before this node is
        // expanded. Ignore that stale body-space collision, but still require
        // the extra clearance above the player's standing height.
        if (actualFeetY.isPresent()) {
            return !world.isBoxColliding(
                centerX - PLAYER_HALF_WIDTH,
                feetY + EPSILON,
                centerZ - PLAYER_HALF_WIDTH,
                centerX + PLAYER_HALF_WIDTH,
                feetY + PLAYER_HEIGHT + MAX_JUMP_RISE,
                centerZ + PLAYER_HALF_WIDTH
            );
        }

        OptionalDouble existingSupport = supportY(world, source.x, source.y, source.z);
        if (existingSupport.isPresent()) {
            // The incoming DIG clears exactly these two source-column cells.
            // Keep checking every neighboring/full-state collision box.
            return !world.isBoxCollidingExcluding(
                centerX - PLAYER_HALF_WIDTH,
                feetY + EPSILON,
                centerZ - PLAYER_HALF_WIDTH,
                centerX + PLAYER_HALF_WIDTH,
                feetY + PLAYER_HEIGHT + MAX_JUMP_RISE,
                centerZ + PLAYER_HALF_WIDTH,
                Set.of(
                    new Vector3i(source.x, source.y, source.z),
                    new Vector3i(source.x, source.y + 1, source.z)
                )
            );
        }

        return !world.isBoxColliding(
            centerX - PLAYER_HALF_WIDTH,
            feetY + EPSILON,
            centerZ - PLAYER_HALF_WIDTH,
            centerX + PLAYER_HALF_WIDTH,
            feetY + PLAYER_HEIGHT + MAX_JUMP_RISE,
            centerZ + PLAYER_HALF_WIDTH
        );
    }

    private static boolean hasDestinationJumpClearance(
            World world, Node target, double sourceFeetY, double targetFeetY) {
        double standingTop = targetFeetY + PLAYER_HEIGHT;
        double jumpTop = Math.max(targetFeetY, sourceFeetY + MAX_JUMP_RISE) + PLAYER_HEIGHT;
        if (jumpTop <= standingTop + EPSILON) return true;

        double centerX = target.x + 0.5;
        double centerZ = target.z + 0.5;
        return !world.isBoxColliding(
            centerX - PLAYER_HALF_WIDTH,
            standingTop + EPSILON,
            centerZ - PLAYER_HALF_WIDTH,
            centerX + PLAYER_HALF_WIDTH,
            jumpTop,
            centerZ + PLAYER_HALF_WIDTH
        );
    }

    /**
     * Resolves the source side of a graph edge after its incoming movement has
     * completed. A source with support but blocked body space represents a DIG
     * node; a source with clear body space but no support represents a
     * BRIDGE/PILLAR node whose support block will have been placed.
     */
    private static OptionalDouble expectedSourceFeetY(World world, Node source) {
        if (!world.chunkLoaded(source.x >> 4, source.z >> 4)) return OptionalDouble.empty();
        OptionalDouble actual = feetY(world, source.x, source.y, source.z);
        if (actual.isPresent()) return actual;

        OptionalDouble existingSupport = supportY(world, source.x, source.y, source.z);
        if (existingSupport.isPresent()) return existingSupport;

        double expectedPlacedSupportY = source.y;
        double centerX = source.x + 0.5;
        double centerZ = source.z + 0.5;
        boolean bodyBlocked = world.isBoxColliding(
            centerX - PLAYER_HALF_WIDTH,
            expectedPlacedSupportY + EPSILON,
            centerZ - PLAYER_HALF_WIDTH,
            centerX + PLAYER_HALF_WIDTH,
            expectedPlacedSupportY + PLAYER_HEIGHT,
            centerZ + PLAYER_HALF_WIDTH
        );
        return bodyBlocked
            ? OptionalDouble.empty()
            : OptionalDouble.of(expectedPlacedSupportY);
    }
}
