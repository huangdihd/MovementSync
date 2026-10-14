package xin.bbtt.pathfinding;

import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.function.BiConsumer;
import org.joml.Vector3d;
import org.joml.Vector3i;
import xin.bbtt.physics.EnvironmentalPhysics;
import xin.bbtt.physics.JumpInputController;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.physics.MovementParameters;
import xin.bbtt.MovementSync;
import xin.bbtt.world.World;

/** Resolves an integer planner node to an exact collision-supported player feet height. */
public final class StandablePositionResolver {
    public static final double PLAYER_HALF_WIDTH = (double) 0.6f / 2;
    public static final double PLAYER_HEIGHT = 1.8;
    public static final double MAX_STEP_RISE = 0.6;
    public static final double MAX_JUMP_RISE = 1.20;
    private static final double EPSILON = 1.0e-7;

    private static final ThreadLocal<SearchCache> SEARCH_CACHE = new ThreadLocal<>();

    private StandablePositionResolver() {}

    /** Query results live only for one synchronous search, never across world updates. */
    static SearchCache beginSearch() {
        return beginSearch(MovementSync.INSTANCE == null
                ? MovementParameters.DEFAULT : MovementSync.INSTANCE.getMovementParameters());
    }

    static SearchCache beginSearch(MovementParameters parameters) {
        SearchCache cache = new SearchCache(SEARCH_CACHE.get(), parameters);
        SEARCH_CACHE.set(cache);
        return cache;
    }

    static final class SearchCache implements AutoCloseable {
        private final SearchCache previous;
        private final MovementParameters parameters;
        private final Map<World, Map<Node, OptionalDouble>> feet = new IdentityHashMap<>();
        private final Map<World, Map<Node, OptionalDouble>> supports = new IdentityHashMap<>();

        private SearchCache(SearchCache previous, MovementParameters parameters) {
            this.previous = previous;
            this.parameters = java.util.Objects.requireNonNull(parameters);
        }

        @Override
        public void close() {
            if (previous == null) SEARCH_CACHE.remove();
            else SEARCH_CACHE.set(previous);
        }
    }

    static MovementParameters movementParameters() {
        SearchCache cache = SEARCH_CACHE.get();
        if (cache != null) return cache.parameters;
        return MovementSync.INSTANCE == null
                ? MovementParameters.DEFAULT : MovementSync.INSTANCE.getMovementParameters();
    }

    /** Candidate bound only; actual edges still require a key-driven physics replay. */
    static double maximumJumpRise() {
        MovementParameters parameters = movementParameters();
        double velocity = parameters.jumpVelocity(1);
        double height = 0;
        for (int tick = 0; tick < 40 && velocity > 0; tick++) {
            height += velocity;
            velocity = parameters.verticalVelocityAfterForces(velocity) * (double) 0.98f;
        }
        return height;
    }

    public static int nodeY(double feetY) {
        if (!Double.isFinite(feetY)) throw new IllegalArgumentException("feetY must be finite");
        return (int) Math.ceil(feetY - 1.0e-6);
    }

    public static OptionalDouble feetY(World world, int x, int nodeY, int z) {
        SearchCache cache = SEARCH_CACHE.get();
        if (cache == null) return resolveFeetY(world, x, nodeY, z);
        return cache.feet.computeIfAbsent(world, ignored -> new HashMap<>())
            .computeIfAbsent(new Node(x, nodeY, z), ignored -> resolveFeetY(world, x, nodeY, z));
    }

    private static OptionalDouble resolveFeetY(World world, int x, int nodeY, int z) {
        OptionalDouble support = supportY(world, x, nodeY, z);
        if (support.isEmpty()) return OptionalDouble.empty();
        double feetY = support.getAsDouble();
        double centerX = x + 0.5;
        double centerZ = z + 0.5;
        // Liquids have no collision shape, but are not safe ground destinations.
        // Submerged floors belong to surface traversal, and lava is never walkable.
        if (intersectsFluid(world, new Vector3d(centerX, feetY, centerZ))
                || !TraversalSafety.isSafe(world, new Vector3d(centerX, feetY, centerZ))) {
            return OptionalDouble.empty();
        }
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

    private static boolean intersectsFluid(World world, Vector3d position) {
        var environment = EnvironmentalPhysics.sample(world, position, PLAYER_HALF_WIDTH, PLAYER_HEIGHT);
        return environment.lavaHeight() > 0 || environment.waterHeight() > 0;
    }

    public static OptionalDouble supportY(World world, int x, int nodeY, int z) {
        SearchCache cache = SEARCH_CACHE.get();
        if (cache == null) return resolveSupportY(world, x, nodeY, z);
        return cache.supports.computeIfAbsent(world, ignored -> new HashMap<>())
            .computeIfAbsent(new Node(x, nodeY, z), ignored -> resolveSupportY(world, x, nodeY, z));
    }

    private static OptionalDouble resolveSupportY(World world, int x, int nodeY, int z) {
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
        return rise.isPresent() && rise.getAsDouble() <= movementParameters().stepHeight() + EPSILON
                && TraversalSafety.isSafeSegment(world,
                    new Vector3d(source.x + 0.5, expectedSourceFeetY(world, source).orElseThrow(), source.z + 0.5),
                    new Vector3d(target.x + 0.5, feetY(world, target.x, target.y, target.z).orElseThrow(), target.z + 0.5));
    }

    public static boolean canJump(World world, Node source, Node target) {
        return canJump(world, source, target, false);
    }

    public static boolean canGapJump(World world, Node source, Node target) {
        return canJump(world, source, target, true);
    }

    private static boolean canJump(
            World world, Node source, Node target, boolean gapJump) {
        return canJump(world, source, target, gapJump, (command, result) -> {});
    }

    // Per-invocation observation keeps tests on the real preview path without shared state.
    static boolean canJump(World world, Node source, Node target, boolean gapJump,
            BiConsumer<JumpInputController.Command, PhysicsResult> observer) {
        OptionalDouble sourceFeetY = expectedSourceFeetY(world, source);
        OptionalDouble targetFeetY = feetY(world, target.x, target.y, target.z);
        if (sourceFeetY.isEmpty() || targetFeetY.isEmpty()) return false;
        double rise = targetFeetY.getAsDouble() - sourceFeetY.getAsDouble();
        return rise <= maximumJumpRise() + EPSILON
            && canCompleteJumpTrajectory(
                world,
                source,
                target,
                sourceFeetY.getAsDouble(),
                targetFeetY.getAsDouble(),
                gapJump,
                observer
            );
    }

    public static boolean canFall(World world, Node source, Node target, double maximumDrop) {
        OptionalDouble rise = rise(world, source, target);
        if (rise.isEmpty()) return false;
        double drop = -rise.getAsDouble();
        return drop > EPSILON && drop <= maximumDrop + EPSILON
                && TraversalSafety.isSafeSegment(world,
                    new Vector3d(source.x + 0.5, expectedSourceFeetY(world, source).orElseThrow(), source.z + 0.5),
                    new Vector3d(target.x + 0.5, feetY(world, target.x, target.y, target.z).orElseThrow(), target.z + 0.5));
    }

    public static boolean canStepToFeetY(World world, Node source, double targetFeetY) {
        return canRiseToFeetY(world, source, targetFeetY, movementParameters().stepHeight());
    }

    public static boolean canJumpToFeetY(World world, Node source, double targetFeetY) {
        return canRiseToFeetY(world, source, targetFeetY, maximumJumpRise())
            && hasJumpClearance(world, source, targetFeetY);
    }

    public static boolean canWalkToSupport(World world, Node source, Node target) {
        return canReachSupport(world, source, target, movementParameters().stepHeight());
    }

    public static boolean canJumpToSupport(World world, Node source, Node target) {
        OptionalDouble targetSupportY = supportY(world, target.x, target.y, target.z);
        return targetSupportY.isPresent()
            && TraversalSafety.isSafe(world, new Vector3d(target.x + 0.5, targetSupportY.getAsDouble(), target.z + 0.5))
            && canReachSupport(world, source, target, maximumJumpRise())
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
            && TraversalSafety.isSafeSegment(world,
                new Vector3d(source.x + 0.5, sourceFeetY.getAsDouble(), source.z + 0.5),
                new Vector3d(target.x + 0.5, targetSupportY.getAsDouble(), target.z + 0.5))
            && !intersectsFluid(world, new Vector3d(
                target.x + 0.5, targetSupportY.getAsDouble(), target.z + 0.5))
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

    /** Replay actual keys and the production physics step; never prescribe a trajectory. */
    private static boolean canCompleteJumpTrajectory(
            World world, Node source, Node target,
            double sourceFeetY, double targetFeetY, boolean gapJump,
            BiConsumer<JumpInputController.Command, PhysicsResult> observer) {
        Vector3d targetPosition = new Vector3d(target.x + 0.5, targetFeetY, target.z + 0.5);
        PhysicsState state = new PhysicsState(
                new Vector3d(source.x + 0.5, sourceFeetY, source.z + 0.5),
                new Vector3d(), true);
        Set<Vector3i> exclusions = jumpSourceExclusions(world, source);
        World collisionWorld = exclusions.isEmpty()
                ? world : new ExcludedBlockCollisionView(world, exclusions);
        MovementParameters parameters = movementParameters();
        for (int tick = 0; tick < 40; tick++) {
            JumpInputController.Command command = JumpInputController.toward(
                    state.position(), targetPosition, state.velocity(), state.onGround(), gapJump);
            PhysicsResult result = new WalkingPhysics(command.input(), command.yaw(), true, parameters).step(
                    collisionWorld, state, PLAYER_HALF_WIDTH, PLAYER_HEIGHT);
            observer.accept(command, result);
            Vector3d position = result.position();
            if (intersectsFluid(world, position)
                    || !TraversalSafety.isSafeSegment(world, state.position(), position)) return false;
            if (result.onGround() && Math.abs(position.y - targetFeetY) <= EPSILON
                    && footprintOverlapsTarget(position, target)) return true;
            if (result.velocity().y < 0
                    && position.y < Math.min(sourceFeetY, targetFeetY) - 1) return false;
            state = new PhysicsState(position, result.velocity(), result.onGround(), result.fallDistance());
        }
        return false;
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

    /**
     * Resolves the source side of a graph edge after its incoming movement has
     * completed. A source with support but blocked body space represents a DIG
     * node; a source with clear body space but no support represents a
     * BRIDGE/PILLAR node whose support block will have been placed.
     */
    private static OptionalDouble expectedSourceFeetY(World world, Node source) {
        if (!world.chunkLoaded(source.x >> 4, source.z >> 4)) return OptionalDouble.empty();
        Vector3d center = new Vector3d(source.x + 0.5, source.y, source.z + 0.5);
        if (EnvironmentalTraversal.position(world, source).isPresent()
                || intersectsFluid(world, center)
                || !TraversalSafety.isSafe(world, center)) return OptionalDouble.empty();
        OptionalDouble actual = feetY(world, source.x, source.y, source.z);
        if (actual.isPresent()) return actual;

        OptionalDouble existingSupport = supportY(world, source.x, source.y, source.z);
        if (existingSupport.isPresent()) {
            center.y = existingSupport.getAsDouble();
            return intersectsFluid(world, center) ? OptionalDouble.empty() : existingSupport;
        }

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
