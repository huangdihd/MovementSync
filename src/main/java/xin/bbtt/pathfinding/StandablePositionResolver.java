package xin.bbtt.pathfinding;

import java.util.OptionalDouble;
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
        OptionalDouble rise = rise(world, source, target);
        return rise.isPresent() && rise.getAsDouble() <= MAX_JUMP_RISE + EPSILON
            && hasJumpClearance(world, source);
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
        OptionalDouble sourceFeetY = feetY(world, source.x, source.y, source.z);
        return sourceFeetY.isPresent()
            && targetFeetY - sourceFeetY.getAsDouble() <= maximumRise + EPSILON;
    }

    private static OptionalDouble rise(World world, Node source, Node target) {
        OptionalDouble sourceFeetY = feetY(world, source.x, source.y, source.z);
        OptionalDouble targetFeetY = feetY(world, target.x, target.y, target.z);
        if (sourceFeetY.isEmpty() || targetFeetY.isEmpty()) return OptionalDouble.empty();
        return OptionalDouble.of(targetFeetY.getAsDouble() - sourceFeetY.getAsDouble());
    }

    private static boolean canReachSupport(
            World world, Node source, Node target, double maximumRise) {
        OptionalDouble sourceFeetY = feetY(world, source.x, source.y, source.z);
        OptionalDouble targetSupportY = supportY(world, target.x, target.y, target.z);
        return sourceFeetY.isPresent() && targetSupportY.isPresent()
            && targetSupportY.getAsDouble() - sourceFeetY.getAsDouble()
                <= maximumRise + EPSILON;
    }

    private static boolean hasJumpClearance(World world, Node source) {
        OptionalDouble sourceFeetY = feetY(world, source.x, source.y, source.z);
        if (sourceFeetY.isEmpty()) return false;
        double centerX = source.x + 0.5;
        double centerZ = source.z + 0.5;
        double feetY = sourceFeetY.getAsDouble();
        return !world.isBoxColliding(
            centerX - PLAYER_HALF_WIDTH,
            feetY + EPSILON,
            centerZ - PLAYER_HALF_WIDTH,
            centerX + PLAYER_HALF_WIDTH,
            feetY + PLAYER_HEIGHT + MAX_JUMP_RISE,
            centerZ + PLAYER_HALF_WIDTH
        );
    }
}
