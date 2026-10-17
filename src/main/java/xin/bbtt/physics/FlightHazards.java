package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;

/** Conservative contact estimates, not a simulation of ignition or damage cooldowns.
 * Fluid cells and hot blocks use their full cell: uncertain edges are not safe routes.
 * Armor, fire resistance and ongoing burning are not modeled by the planning snapshot.
 */
public final class FlightHazards {
    private FlightHazards() {}

    /** Shared landing filter; check standing clearance for hazards as well as hot support. */
    public static boolean safeLanding(World world, Vector3d feet) {
        return contact(world, feet, feet, 1.8, true) == 0
                && contact(world, feet, feet, 1.8, false) == 0;
    }

    static DamageModel.Damage damage(World world, PhysicsState before, PhysicsResult after,
            DamageParameters mitigation, boolean lava) {
        float raw = contact(world, before.position(), after.position(),
                Math.max(before.pose().height(), after.pose().height()), lava);
        // These sources do not bypass armor. Until armor/fire protection are in the snapshot,
        // deliberately grant only known general protection/resistance (a conservative estimate).
        return DamageModel.apply(raw, DamageModel.Source.WALL, mitigation);
    }

    private static float contact(World world, Vector3d from, Vector3d to, double height, boolean lava) {
        float raw = 0;
        double radius = PlayerPose.HALF_WIDTH;
        for (int x = (int) Math.floor(Math.min(from.x, to.x) - radius);
                x <= (int) Math.floor(Math.max(from.x, to.x) + radius); x++) {
            for (int y = (int) Math.floor(Math.min(from.y, to.y) - 1.0e-7);
                    y <= (int) Math.floor(Math.max(from.y, to.y) + height - 1.0e-7); y++) {
                for (int z = (int) Math.floor(Math.min(from.z, to.z) - radius);
                        z <= (int) Math.floor(Math.max(from.z, to.z) + radius); z++) {
                    if (!intersects(from, to, x - radius, y - height + 1.0e-7, z - radius,
                            x + 1 + radius, y + 1 + 1.0e-7, z + 1 + radius)) continue;
                    BlockState block = world.getBlockStateAt(new Vector3d(x, y, z));
                    String name = block.blockName();
                    if (name == null) continue;
                    name = name.replace("minecraft:", "");
                    float damage = lava ? switch (name) {
                        case "lava", "lava_cauldron" -> 4;
                        default -> 0;
                    } : switch (name) {
                        case "fire", "magma_block" -> 1;
                        case "soul_fire" -> 2;
                        case "campfire" -> "false".equals(block.getProperty("lit")) ? 0 : 1;
                        case "soul_campfire" -> "false".equals(block.getProperty("lit")) ? 0 : 2;
                        default -> 0;
                    };
                    // Multiple cells of one source are not independent damage events.
                    raw = Math.max(raw, damage);
                }
            }
        }
        return raw;
    }

    private static boolean intersects(Vector3d from, Vector3d to, double minX, double minY,
            double minZ, double maxX, double maxY, double maxZ) {
        double enter = 0, exit = 1;
        for (int axis = 0; axis < 3; axis++) {
            double start = from.get(axis), delta = to.get(axis) - start;
            double min = axis == 0 ? minX : axis == 1 ? minY : minZ;
            double max = axis == 0 ? maxX : axis == 1 ? maxY : maxZ;
            if (delta == 0) {
                if (start <= min || start >= max) return false;
            } else {
                double a = (min - start) / delta, b = (max - start) / delta;
                enter = Math.max(enter, Math.min(a, b));
                exit = Math.min(exit, Math.max(a, b));
                if (enter > exit) return false;
            }
        }
        return true;
    }
}
