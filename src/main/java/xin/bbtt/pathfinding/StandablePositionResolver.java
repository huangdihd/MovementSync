package xin.bbtt.pathfinding;

import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3d;
import org.joml.Vector3i;
import xin.bbtt.MovementSync;
import xin.bbtt.tasks.VerticalCollisionResolver;
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
        return canJump(world, source, target, 1.0);
    }

    public static boolean canGapJump(World world, Node source, Node target) {
        return canJump(world, source, target, MovementSync.gapJumpSpeedMultiplier);
    }

    private static boolean canJump(
            World world, Node source, Node target, double horizontalSpeedMultiplier) {
        OptionalDouble sourceFeetY = expectedSourceFeetY(world, source);
        OptionalDouble targetFeetY = feetY(world, target.x, target.y, target.z);
        if (sourceFeetY.isEmpty() || targetFeetY.isEmpty()) return false;
        double rise = targetFeetY.getAsDouble() - sourceFeetY.getAsDouble();
        return rise <= MAX_JUMP_RISE + EPSILON
            && canCompleteJumpTrajectory(
                world,
                source,
                target,
                sourceFeetY.getAsDouble(),
                targetFeetY.getAsDouble(),
                horizontalSpeedMultiplier
            );
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
            && hasJumpClearance(world, source, targetFeetY);
    }

    public static boolean canWalkToSupport(World world, Node source, Node target) {
        return canReachSupport(world, source, target, MAX_STEP_RISE);
    }

    public static boolean canJumpToSupport(World world, Node source, Node target) {
        OptionalDouble targetSupportY = supportY(world, target.x, target.y, target.z);
        return targetSupportY.isPresent()
            && canReachSupport(world, source, target, MAX_JUMP_RISE)
            && hasJumpClearance(world, source, targetSupportY.getAsDouble());
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

    private static boolean hasJumpClearance(World world, Node source, double requiredFeetY) {
        OptionalDouble actualFeetY = feetY(world, source.x, source.y, source.z);
        OptionalDouble sourceFeetY = actualFeetY.isPresent()
            ? actualFeetY
            : expectedSourceFeetY(world, source);
        if (sourceFeetY.isEmpty()) return false;
        double centerX = source.x + 0.5;
        double centerZ = source.z + 0.5;
        double feetY = sourceFeetY.getAsDouble();
        double requiredTop = Math.max(feetY, requiredFeetY) + PLAYER_HEIGHT;
        if (actualFeetY.isPresent()) {
            return !world.isBoxColliding(
                centerX - PLAYER_HALF_WIDTH,
                feetY + EPSILON,
                centerZ - PLAYER_HALF_WIDTH,
                centerX + PLAYER_HALF_WIDTH,
                requiredTop,
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
                requiredTop,
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
            requiredTop,
            centerZ + PLAYER_HALF_WIDTH
        );
    }

    /**
     * Replays a cardinal jump with the same horizontal input, gravity, drag,
     * and axis-separated collision order as PathMovement and updateMotionTask.
     * A ceiling hit zeros upward velocity but does not fail the edge when the
     * remaining trajectory can still get over the target support.
     */
    private static boolean canCompleteJumpTrajectory(
            World world,
            Node source,
            Node target,
            double sourceFeetY,
            double targetFeetY,
            double horizontalSpeedMultiplier) {
        Vector3d position = new Vector3d(source.x + 0.5, sourceFeetY, source.z + 0.5);
        double targetX = target.x + 0.5;
        double targetZ = target.z + 0.5;
        double velocityY = MovementSync.jumpVelocity;
        Set<Vector3i> excludedBlocks = jumpSourceExclusions(world, source);
        double tickStartY = sourceFeetY;

        for (int tick = 0; tick < 40; tick++) {
            double dx = targetX - position.x;
            double dz = targetZ - position.z;
            double horizontalDistance = Math.sqrt(dx * dx + dz * dz);
            double velocityX = 0.0;
            double velocityZ = 0.0;
            if (horizontalDistance < 0.05) {
                position.x = targetX;
                position.z = targetZ;
            } else if (horizontalDistance > EPSILON) {
                double speed = MovementSync.movementSpeed * horizontalSpeedMultiplier;
                if (horizontalDistance < 0.25) speed *= horizontalDistance / 0.25;
                velocityX = dx / horizontalDistance * speed;
                velocityZ = dz / horizontalDistance * speed;
            }

            if (tick > 0) velocityY += MovementSync.gravitationalAcceleration.y;
            velocityY *= MovementSync.verticalDrag;

            if (velocityX != 0.0) {
                Vector3d candidate = new Vector3d(position.x + velocityX, position.y, position.z);
                if (!isJumpBoxColliding(world, candidate, excludedBlocks)) {
                    position.x = candidate.x;
                } else if (tick == 0) {
                    tryJumpStepUp(
                        world, position, candidate.x, candidate.z, tickStartY, excludedBlocks);
                }
            }
            if (velocityZ != 0.0) {
                Vector3d candidate = new Vector3d(position.x, position.y, position.z + velocityZ);
                if (!isJumpBoxColliding(world, candidate, excludedBlocks)) {
                    position.z = candidate.z;
                } else if (tick == 0) {
                    tryJumpStepUp(
                        world, position, candidate.x, candidate.z, tickStartY, excludedBlocks);
                }
            }

            if (velocityY > 0.0) {
                Vector3d candidate = new Vector3d(position.x, position.y + velocityY, position.z);
                if (isJumpBoxColliding(world, candidate, excludedBlocks)) {
                    velocityY = 0.0;
                } else {
                    position.y = candidate.y;
                }
            } else {
                double desiredY = position.y + velocityY;
                double resolvedY = VerticalCollisionResolver.resolveDownwardY(
                    world, position, PLAYER_HALF_WIDTH, velocityY, excludedBlocks);
                position.y = resolvedY;
                if (resolvedY > desiredY + EPSILON) {
                    velocityY = 0.0;
                    if (Math.abs(resolvedY - targetFeetY) <= EPSILON
                            && footprintOverlapsTarget(position, target)) {
                        return true;
                    }
                }
            }

            if (velocityY < 0.0 && position.y < Math.min(sourceFeetY, targetFeetY) - 1.0) {
                return false;
            }
        }
        return false;
    }

    private static boolean tryJumpStepUp(
            World world,
            Vector3d position,
            double candidateX,
            double candidateZ,
            double tickStartY,
            Set<Vector3i> excludedBlocks) {
        OptionalDouble support = world.findHighestCollisionTopExcluding(
            candidateX - PLAYER_HALF_WIDTH,
            position.y + EPSILON,
            candidateZ - PLAYER_HALF_WIDTH,
            candidateX + PLAYER_HALF_WIDTH,
            tickStartY + MAX_STEP_RISE,
            candidateZ + PLAYER_HALF_WIDTH,
            excludedBlocks
        );
        if (support.isEmpty()) return false;
        Vector3d stepped = new Vector3d(candidateX, support.getAsDouble(), candidateZ);
        if (isJumpBoxColliding(world, stepped, excludedBlocks)) return false;
        position.set(stepped);
        return true;
    }

    private static boolean footprintOverlapsTarget(Vector3d position, Node target) {
        return position.x + PLAYER_HALF_WIDTH > target.x + EPSILON
            && position.x - PLAYER_HALF_WIDTH < target.x + 1.0 - EPSILON
            && position.z + PLAYER_HALF_WIDTH > target.z + EPSILON
            && position.z - PLAYER_HALF_WIDTH < target.z + 1.0 - EPSILON;
    }

    private static Set<Vector3i> jumpSourceExclusions(World world, Node source) {
        if (feetY(world, source.x, source.y, source.z).isPresent()
                || supportY(world, source.x, source.y, source.z).isEmpty()) {
            return Set.of();
        }
        return Set.of(
            new Vector3i(source.x, source.y, source.z),
            new Vector3i(source.x, source.y + 1, source.z)
        );
    }

    private static boolean isJumpBoxColliding(
            World world, Vector3d feetPosition, Set<Vector3i> excludedBlocks) {
        return world.isBoxCollidingExcluding(
            feetPosition.x - PLAYER_HALF_WIDTH,
            feetPosition.y,
            feetPosition.z - PLAYER_HALF_WIDTH,
            feetPosition.x + PLAYER_HALF_WIDTH,
            feetPosition.y + PLAYER_HEIGHT,
            feetPosition.z + PLAYER_HALF_WIDTH,
            excludedBlocks
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
