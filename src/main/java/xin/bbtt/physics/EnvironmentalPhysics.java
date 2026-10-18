package xin.bbtt.physics;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.joml.Vector3d;
import xin.bbtt.Block.BlockState;
import xin.bbtt.collision.CollisionBox;
import xin.bbtt.collision.CollisionShapeRegistry;
import xin.bbtt.world.World;

import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

/** Vanilla 1.21.11 player fluid and climbing sampling, without velocity-dependent push flooring. */
public final class EnvironmentalPhysics {
    private static final int[][] HORIZONTAL = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    private static final Set<String> CLIMBABLE = values(resource("data/minecraft/tags/block/climbable.json"), "values");
    // Generated from Blocks settings; collision bounds alone cannot recover these overrides.
    private static final JsonObject BLOCK_PROPERTIES = resource("environment_block_properties.json");
    private static final Set<String> FORCE_SOLID = values(BLOCK_PROPERTIES, "force_solid");
    private static final Set<String> FORCE_NOT_SOLID = values(BLOCK_PROPERTIES, "force_not_solid");
    private static final Set<String> DYNAMIC_BOUNDS = values(BLOCK_PROPERTIES, "dynamic_bounds");
    private static final Set<String> WATER_PLANTS = Set.of("kelp", "kelp_plant", "seagrass", "tall_seagrass", "bubble_column");

    private EnvironmentalPhysics() {}

    public record Environment(double waterHeight, double lavaHeight, Vector3d waterPush,
                              Vector3d lavaPush, boolean climbing, boolean scaffolding) {}

    public static Environment sample(World world, Vector3d position, double halfWidth, double height) {
        int feetX = floor(position.x), feetY = floor(position.y), feetZ = floor(position.z);
        BlockState feet = block(world, feetX, feetY, feetZ);
        String feetName = name(feet);
        boolean climbing = CLIMBABLE.contains(feetName);
        if (!climbing && feetName.endsWith("_trapdoor") && "true".equals(feet.getProperty("open"))) {
            BlockState below = block(world, feetX, feetY - 1, feetZ);
            climbing = name(below).equals("ladder")
                    && Objects.equals(feet.getProperty("facing"), below.getProperty("facing"));
        }
        boolean scaffolding = feetName.equals("scaffolding");
        if (!regionLoaded(world, position, halfWidth)) {
            return new Environment(0, 0, new Vector3d(), new Vector3d(), climbing, scaffolding);
        }
        double minimumY = position.y + 0.001;
        double[] depths = new double[3];
        int[] counts = new int[3];
        Vector3d[] pushes = {new Vector3d(), new Vector3d(), new Vector3d()};
        // Preserve Entity's x/y/z order: shallow scaling uses the running maximum depth.
        for (int x = floor(position.x - halfWidth + 0.001); x < Math.ceil(position.x + halfWidth - 0.001); x++) {
            for (int y = floor(minimumY); y < Math.ceil(position.y + height - 0.001); y++) {
                for (int z = floor(position.z - halfWidth + 0.001); z < Math.ceil(position.z + halfWidth - 0.001); z++) {
                    Fluid fluid = fluid(block(world, x, y, z));
                    if (fluid.type == 0) continue;
                    double surface = y + (fluid(block(world, x, y + 1, z)).type == fluid.type ? 1.0F : fluid.height);
                    if (surface < minimumY) continue;
                    depths[fluid.type] = Math.max(depths[fluid.type], surface - minimumY);
                    Vector3d velocity = flow(world, x, y, z, fluid);
                    if (depths[fluid.type] < 0.4) velocity.mul(depths[fluid.type]);
                    pushes[fluid.type].add(velocity);
                    counts[fluid.type]++;
                }
            }
        }
        if (counts[1] > 0) pushes[1].mul(1.0 / counts[1]).mul(0.014);
        if (counts[2] > 0) pushes[2].mul(1.0 / counts[2]).mul(0.0023333333333333335);
        return new Environment(depths[1], depths[2], pushes[1], pushes[2], climbing, scaffolding);
    }

    public static BlockMovementProperties supportProperties(World world, Vector3d position) {
        return BlockMovementProperties.of(block(world, floor(position.x),
                floor(position.y - (double) 0.500001f), floor(position.z)));
    }

    public static double jumpMultiplier(World world, Vector3d position) {
        double multiplier = BlockMovementProperties.of(block(world, floor(position.x),
                floor(position.y), floor(position.z))).jumpMultiplier();
        return multiplier == 1 ? supportProperties(world, position).jumpMultiplier() : multiplier;
    }

    public static double velocityMultiplier(World world, Vector3d position) {
        BlockMovementProperties feet = BlockMovementProperties.of(block(world, floor(position.x),
                floor(position.y), floor(position.z)));
        return feet.velocityMultiplier() != 1 || feet.ignoresSupportVelocity()
                ? feet.velocityMultiplier() : supportProperties(world, position).velocityMultiplier();
    }

    /** Entity.getVelocityMultiplier uses the supporting block's X/Z, not always the feet center. */
    public static double velocityMultiplier(World world, Vector3d position, double halfWidth,
                                            boolean grounded, Vector3d movement) {
        BlockMovementProperties feet = BlockMovementProperties.of(block(world, floor(position.x),
                floor(position.y), floor(position.z)));
        if (feet.velocityMultiplier() != 1 || feet.ignoresSupportVelocity()) return feet.velocityMultiplier();
        Vector3d support = grounded ? supportingBlock(world, position, position, halfWidth) : null;
        if (grounded && support == null) {
            // Entity.updateSupportingBlockPos retries the pre-horizontal-move footprint at an edge.
            support = supportingBlock(world, position,
                    new Vector3d(position).sub(movement.x, 0, movement.z), halfWidth);
        }
        return support == null ? supportProperties(world, position).velocityMultiplier()
                : BlockMovementProperties.of(block(world, floor(support.x),
                        floor(position.y - (double) 0.500001f), floor(support.z))).velocityMultiplier();
    }

    private static Vector3d supportingBlock(World world, Vector3d position, Vector3d footprint, double halfWidth) {
        CollisionShapeRegistry registry = CollisionShapeRegistry.getInstance();
        Vector3d nearest = null;
        double distance = Double.POSITIVE_INFINITY;
        // Vanilla probes a 1e-6-deep slice below the feet and picks the nearest
        // block center (ties: greatest BlockPos, ordered Y/Z/X).
        for (int y = floor(footprint.y - 1.0e-6 - registry.maxY()) + 1;
                y <= Math.ceil(footprint.y - registry.minY()) - 1; y++) {
            for (int z = floor(footprint.z - halfWidth - registry.maxZ()) + 1;
                    z <= Math.ceil(footprint.z + halfWidth - registry.minZ()) - 1; z++) {
                for (int x = floor(footprint.x - halfWidth - registry.maxX()) + 1;
                        x <= Math.ceil(footprint.x + halfWidth - registry.minX()) - 1; x++) {
                    if (!registry.intersects(block(world, x, y, z).stateId(), x, y, z,
                            footprint.x - halfWidth, footprint.y - 1.0e-6, footprint.z - halfWidth,
                            footprint.x + halfWidth, footprint.y, footprint.z + halfWidth)) continue;
                    double candidate = position.distanceSquared(x + 0.5, y + 0.5, z + 0.5);
                    if (candidate <= distance) {
                        distance = candidate;
                        nearest = new Vector3d(x, y, z);
                    }
                }
            }
        }
        return nearest;
    }

    public static BlockMovementProperties landingProperties(World world, Vector3d position) {
        return BlockMovementProperties.of(block(world, floor(position.x),
                floor(position.y - (double) 0.2f), floor(position.z)));
    }

    /** Sample tick-start contacts without adding mutable block state to the pure stepper. */
    public static Vector3d stuckMultiplier(World world, Vector3d position, double halfWidth, double height) {
        Vector3d multiplier = new Vector3d(1);
        forEachContact(world, position, halfWidth, height, (state, x, y, z) -> {
            BlockMovementProperties properties = BlockMovementProperties.of(state);
            if (properties.requiresFeetContact() && !BlockMovementProperties.of(block(world,
                    floor(position.x), floor(position.y), floor(position.z))).requiresFeetContact()) return;
            if (properties.stuck()) multiplier.set(properties.stuckHorizontal(),
                    properties.stuckVertical(), properties.stuckHorizontal());
        });
        return multiplier;
    }

    /** Collision callbacks run after travel drag/gravity, as in LivingEntity.tickMovement. */
    public static void applyContactVelocity(World world, Vector3d position, Vector3d velocity,
                                           double halfWidth, double height, boolean onGround) {
        forEachContact(world, position, halfWidth, height, (state, x, y, z) -> {
            BlockMovementProperties properties = BlockMovementProperties.of(state);
            if (properties.honeySliding() && !onGround && position.y <= y + 0.9375 - 1.0e-7
                    && (Math.abs(x + 0.5 - position.x) + 1.0e-7 > 0.4375 + halfWidth
                    || Math.abs(z + 0.5 - position.z) + 1.0e-7 > 0.4375 + halfWidth)) {
                applyHoneySlide(velocity);
            }
            if (properties.bubbleColumn()) {
                BlockState above = block(world, x, y + 1, z);
                boolean surface = fluid(above).type == 0
                        && CollisionShapeRegistry.getInstance().boxesFor(above.stateId()).isEmpty();
                velocity.y = bubbleVelocity(velocity.y, !"false".equals(state.getProperty("drag")), surface);
            }
        });
    }

    static void applyHoneySlide(Vector3d velocity) {
        double previousY = velocity.y / (double) 0.98f + 0.08;
        if (previousY >= -0.08) return;
        if (previousY < -0.13) {
            double scale = -0.05 / previousY;
            velocity.x *= scale;
            velocity.z *= scale;
        }
        velocity.y = (-0.05 - 0.08) * (double) 0.98f;
    }

    static double bubbleVelocity(double velocityY, boolean drag, boolean surface) {
        if (drag) return Math.max(surface ? -0.9 : -0.3, velocityY - 0.03);
        return Math.min(surface ? 1.8 : 0.7, velocityY + (surface ? 0.1 : 0.06));
    }

    private interface ContactConsumer {
        void accept(BlockState state, int x, int y, int z);
    }

    private static void forEachContact(World world, Vector3d position, double halfWidth, double height,
                                       ContactConsumer consumer) {
        for (int x = floor(position.x - halfWidth + 1.0e-7); x < Math.ceil(position.x + halfWidth - 1.0e-7); x++) {
            for (int y = floor(position.y + 1.0e-7); y < Math.ceil(position.y + height - 1.0e-7); y++) {
                for (int z = floor(position.z - halfWidth + 1.0e-7); z < Math.ceil(position.z + halfWidth - 1.0e-7); z++) {
                    if (world.chunkLoaded(x >> 4, z >> 4)) consumer.accept(block(world, x, y, z), x, y, z);
                }
            }
        }
    }

    private static boolean regionLoaded(World world, Vector3d position, double halfWidth) {
        int minimumX = floor(position.x - halfWidth - 1) >> 4;
        int maximumX = (int) Math.ceil(position.x + halfWidth + 1) >> 4;
        int minimumZ = floor(position.z - halfWidth - 1) >> 4;
        int maximumZ = (int) Math.ceil(position.z + halfWidth + 1) >> 4;
        for (int x = minimumX; x <= maximumX; x++) {
            for (int z = minimumZ; z <= maximumZ; z++) {
                if (!world.chunkLoaded(x, z)) return false;
            }
        }
        return true;
    }

    private record Fluid(int type, float height, boolean falling) {}

    private static Fluid fluid(BlockState block) {
        String name = name(block);
        if (name.equals("water") || name.equals("lava")) {
            String property = block.getProperty("level");
            int level = property == null ? 0 : Integer.parseInt(property);
            return new Fluid(name.equals("water") ? 1 : 2, (level >= 8 ? 8 : 8 - level) / 9.0F, level >= 8);
        }
        if ("true".equals(block.getProperty("waterlogged")) || WATER_PLANTS.contains(name)) {
            return new Fluid(1, 8 / 9.0F, false);
        }
        return new Fluid(0, 0, false);
    }

    private static Vector3d flow(World world, int x, int y, int z, Fluid current) {
        Vector3d velocity = new Vector3d();
        for (int[] direction : HORIZONTAL) {
            BlockState neighbor = block(world, x + direction[0], y, z + direction[1]);
            Fluid adjacent = fluid(neighbor);
            if (adjacent.type != 0 && adjacent.type != current.type) continue;
            float difference = 0;
            if (adjacent.height == 0) {
                if (!blocksMovement(neighbor)) {
                    Fluid below = fluid(block(world, x + direction[0], y - 1, z + direction[1]));
                    if ((below.type == 0 || below.type == current.type) && below.height > 0) {
                        difference = current.height - (below.height - 0.8888889F);
                    }
                }
            } else {
                difference = current.height - adjacent.height;
            }
            velocity.add(direction[0] * difference, 0, direction[1] * difference);
        }
        if (current.falling) {
            for (int[] direction : HORIZONTAL) {
                if (flowBlocked(world, x + direction[0], y, z + direction[1], direction, current.type)
                        || flowBlocked(world, x + direction[0], y + 1, z + direction[1], direction, current.type)) {
                    normalize(velocity).add(0, -6, 0);
                    break;
                }
            }
        }
        return normalize(velocity);
    }

    private static Vector3d normalize(Vector3d vector) {
        double length = vector.length();
        return length < 1.0E-5 ? vector.zero() : vector.div(length);
    }

    private static boolean blocksMovement(BlockState block) {
        String name = name(block);
        if (name.equals("cobweb") || name.equals("bamboo_sapling")) return false;
        if (FORCE_SOLID.contains(name)) return true;
        if (FORCE_NOT_SOLID.contains(name) || DYNAMIC_BOUNDS.contains(name)) return false;
        List<CollisionBox> boxes = CollisionShapeRegistry.getInstance().boxesFor(block.stateId());
        if (boxes.isEmpty()) return false;
        double minimumX = Double.POSITIVE_INFINITY, minimumY = minimumX, minimumZ = minimumX;
        double maximumX = Double.NEGATIVE_INFINITY, maximumY = maximumX, maximumZ = maximumX;
        for (CollisionBox box : boxes) {
            minimumX = Math.min(minimumX, box.minX()); minimumY = Math.min(minimumY, box.minY()); minimumZ = Math.min(minimumZ, box.minZ());
            maximumX = Math.max(maximumX, box.maxX()); maximumY = Math.max(maximumY, box.maxY()); maximumZ = Math.max(maximumZ, box.maxZ());
        }
        return ((maximumX - minimumX) + (maximumY - minimumY) + (maximumZ - minimumZ)) / 3 >= 0.7291666666666666
                || maximumY - minimumY >= 1;
    }

    private static boolean flowBlocked(World world, int x, int y, int z, int[] direction, int type) {
        BlockState block = block(world, x, y, z);
        // IceBlock includes frosted ice, but not packed or blue ice.
        if (fluid(block).type == type || name(block).equals("ice") || name(block).equals("frosted_ice")) return false;
        String name = name(block);
        // These blocks override getSidesShape rather than using their collision shape.
        if (name.endsWith("_leaves") || name.equals("chorus_flower") || name.endsWith("_fence_gate")
                || name.endsWith("_hanging_sign")) return false;
        if (name.equals("mud") || name.equals("soul_sand") || name.endsWith("shulker_box")) return true;
        if (name.equals("snow")) return "8".equals(block.getProperty("layers"));
        List<double[]> uncovered = new ArrayList<>();
        uncovered.add(new double[]{0, 0, 1, 1});
        for (CollisionBox box : CollisionShapeRegistry.getInstance().boxesFor(block.stateId())) {
            boolean xFace = direction[0] != 0;
            double minimum = xFace ? box.minX() : box.minZ();
            double maximum = xFace ? box.maxX() : box.maxZ();
            boolean positive = direction[0] + direction[1] > 0;
            double face = positive ? maximum : minimum;
            if (Math.abs(face - (positive ? 1 : 0)) > 1.0E-7) continue;
            double left = xFace ? box.minZ() : box.minX(), right = xFace ? box.maxZ() : box.maxX();
            List<double[]> remaining = new ArrayList<>();
            for (double[] rectangle : uncovered) {
                double a = Math.max(rectangle[0], left), b = Math.max(rectangle[1], box.minY());
                double c = Math.min(rectangle[2], right), d = Math.min(rectangle[3], box.maxY());
                if (a >= c || b >= d) { remaining.add(rectangle); continue; }
                if (rectangle[0] < a) remaining.add(new double[]{rectangle[0], rectangle[1], a, rectangle[3]});
                if (c < rectangle[2]) remaining.add(new double[]{c, rectangle[1], rectangle[2], rectangle[3]});
                if (rectangle[1] < b) remaining.add(new double[]{a, rectangle[1], c, b});
                if (d < rectangle[3]) remaining.add(new double[]{a, d, c, rectangle[3]});
            }
            uncovered = remaining;
            if (uncovered.isEmpty()) return true;
        }
        return false;
    }

    private static BlockState block(World world, int x, int y, int z) {
        return world.getBlockStateAt(new Vector3d(x, y, z));
    }

    private static int floor(double value) { return (int) Math.floor(value); }
    private static String name(BlockState block) { return block.blockName().replace("minecraft:", ""); }

    private static JsonObject resource(String path) {
        try (InputStreamReader reader = new InputStreamReader(Objects.requireNonNull(
                EnvironmentalPhysics.class.getClassLoader().getResourceAsStream(path), "Missing " + path), StandardCharsets.UTF_8)) {
            return JsonParser.parseReader(reader).getAsJsonObject();
        } catch (IOException error) {
            throw new ExceptionInInitializerError(error);
        }
    }

    private static Set<String> values(JsonObject object, String key) {
        Set<String> values = new HashSet<>();
        object.getAsJsonArray(key).forEach(value -> values.add(value.getAsString().replace("minecraft:", "")));
        return Set.copyOf(values);
    }
}
