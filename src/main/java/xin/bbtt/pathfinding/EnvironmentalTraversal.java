package xin.bbtt.pathfinding;

import java.util.Optional;
import org.joml.Vector3d;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.EnvironmentalPhysics;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

/** Shared environmental steering for route preview and path execution. */
public final class EnvironmentalTraversal {
    private static final double HALF_WIDTH = StandablePositionResolver.PLAYER_HALF_WIDTH;
    private static final double HEIGHT = StandablePositionResolver.PLAYER_HEIGHT;

    private EnvironmentalTraversal() {}

    public record Command(PlayerInput input, float yaw) {}

    public static boolean isEnvironmental(MovementType type) {
        return type == BuiltinMovementType.SWIM || type == BuiltinMovementType.CLIMB;
    }

    public static boolean inWater(World world, Vector3d position) {
        return EnvironmentalPhysics.sample(world, position, HALF_WIDTH, HEIGHT).waterHeight() > 0;
    }

    public static Optional<Vector3d> position(World world, Node node) {
        if (!world.chunkLoaded(node.x >> 4, node.z >> 4)) return Optional.empty();
        Vector3d center = new Vector3d(node.x + 0.5, node.y, node.z + 0.5);
        if (isClimbable(world.getBlockStateAt(center))) {
            return clear(world, center) ? Optional.of(center) : Optional.empty();
        }
        BlockState below = world.getBlockStateAt(new Vector3d(center).add(0, -1, 0));
        BlockState above = world.getBlockStateAt(center);
        // Only exposed water surfaces, never submerged tunnels or lava routes.
        if (isWater(below) && !isWater(above)) {
            String level = below.getProperty("level");
            int depth = level == null ? 0 : Integer.parseInt(level);
            double surface = node.y - (depth >= 8 ? 1.0 / 9 : (depth + 1.0) / 9);
            center.y = surface - 0.25;
            return clear(world, center) ? Optional.of(center) : Optional.empty();
        }
        return Optional.empty();
    }

    /** Recovery is vertical only and requires an unobstructed column to open water. */
    public static Optional<Vector3d> recoveryPosition(World world, Node node) {
        if (!world.chunkLoaded(node.x >> 4, node.z >> 4)) return Optional.empty();
        Vector3d center = new Vector3d(node.x + 0.5, node.y, node.z + 0.5);
        if (!isWater(world.getBlockStateAt(center)) || !clear(world, center)) return Optional.empty();
        for (int y = node.y + 1; y <= World.getMaxWorldY(); y++) {
            Vector3d above = new Vector3d(center.x, y, center.z);
            if (!clear(world, above)) return Optional.empty();
            if (!isWater(world.getBlockStateAt(above))) {
                return position(world, new Node(node.x, y, node.z)).isPresent()
                        ? Optional.of(center) : Optional.empty();
            }
        }
        return Optional.empty();
    }

    public static Optional<Vector3d> destination(World world, Node node) {
        Optional<Vector3d> environmental = position(world, node);
        if (environmental.isPresent()) return environmental;
        Optional<Vector3d> recovery = recoveryPosition(world, node);
        if (recovery.isPresent()) return recovery;
        var feet = StandablePositionResolver.feetY(world, node.x, node.y, node.z);
        return feet.isPresent()
                ? Optional.of(new Vector3d(node.x + 0.5, feet.getAsDouble(), node.z + 0.5))
                : Optional.empty();
    }

    public static boolean climbing(World world, Vector3d position) {
        return EnvironmentalPhysics.sample(world, position, HALF_WIDTH, HEIGHT).climbing();
    }

    public static Command toward(World world, Vector3d position, Vector3d target,
            Vector3d velocity, boolean onGround) {
        double deltaX = target.x - position.x;
        double deltaZ = target.z - position.z;
        double distance = Math.hypot(deltaX, deltaZ);
        float yaw = (float) Math.toDegrees(Math.atan2(-deltaX, deltaZ));
        boolean water = inWater(world, position);
        boolean climb = climbing(world, position);
        double closingSpeed = distance > 0 ? (velocity.x * deltaX + velocity.z * deltaZ) / distance : 0;
        double drag = onGround ? 0.546 : 0.91;
        if (water) drag = 0.8;
        boolean forward = distance > Math.max(0.08, closingSpeed / (1 - drag));
        boolean jump = onGround && target.y > position.y + 0.6;
        if (water) jump = position.y < target.y + 0.08;
        else if (climb) jump = target.y > position.y + 0.08;
        // Sneak holds a ladder at its destination, but must not inhibit descent.
        boolean sneak = climb && Math.abs(target.y - position.y) < 0.08 && distance < 0.12;
        return new Command(new PlayerInput(forward, false, false, false, jump, sneak, false), yaw);
    }

    public static boolean canTraverse(World world, Node source, Node target) {
        Optional<Vector3d> start = destination(world, source);
        Optional<Vector3d> end = destination(world, target);
        if (start.isEmpty() || end.isEmpty()) return false;
        PhysicsState state = new PhysicsState(start.get(), new Vector3d(), world.isOnGround(start.get()));
        var parameters = StandablePositionResolver.movementParameters();
        for (int tick = 0; tick < 100; tick++) {
            Command command = toward(world, state.position(), end.get(), state.velocity(), state.onGround());
            PhysicsResult result = new WalkingPhysics(command.input(), command.yaw(), true, parameters)
                    .step(world, state, HALF_WIDTH, HEIGHT);
            Vector3d position = result.position();
            if (!world.chunkLoaded((int) Math.floor(position.x) >> 4, (int) Math.floor(position.z) >> 4)) return false;
            if (!TraversalSafety.isSafeSegment(world, state.position(), position)) return false;
            if (Math.abs(position.x - end.get().x) < 0.25
                    && Math.abs(position.z - end.get().z) < 0.25
                    && Math.abs(position.y - end.get().y) < 0.18) return true;
            if (position.y < Math.min(start.get().y, end.get().y) - 1.5) return false;
            state = new PhysicsState(position, result.velocity(), result.onGround(), result.fallDistance());
        }
        return false;
    }

    private static boolean clear(World world, Vector3d position) {
        return TraversalSafety.isSafe(world, position)
                && !world.isBoxColliding(position.x - HALF_WIDTH, position.y + 1.0e-7,
                position.z - HALF_WIDTH, position.x + HALF_WIDTH, position.y + HEIGHT,
                position.z + HALF_WIDTH);
    }

    private static boolean isWater(BlockState state) {
        return "water".equals(state.blockName()) || "bubble_column".equals(state.blockName());
    }

    private static boolean isClimbable(BlockState state) {
        String name = state.blockName();
        return "ladder".equals(name) || "vine".equals(name)
                || "weeping_vines".equals(name) || "weeping_vines_plant".equals(name)
                || "twisting_vines".equals(name) || "twisting_vines_plant".equals(name);
    }
}
