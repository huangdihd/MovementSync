package xin.bbtt.physics;

import java.util.OptionalDouble;
import org.joml.Vector3d;
import xin.bbtt.listeners.RegistryDataListener;
import xin.bbtt.world.World;

/** Loaded collision geometry in ceiling dimensions; logical_height is not a roof. */
public final class CeilingCollision {
    private CeilingCollision() {}

    /** Highest actual collision face in the feet footprint, never a guessed Y level. */
    public static OptionalDouble roofTop(World world, double x, double z) {
        if (!RegistryDataListener.hasCeiling()) return OptionalDouble.empty();
        double width = VanillaGlideMode.HALF_WIDTH;
        for (int cx = (int) Math.floor(x - width) >> 4; cx <= (int) Math.floor(x + width) >> 4; cx++)
            for (int cz = (int) Math.floor(z - width) >> 4; cz <= (int) Math.floor(z + width) >> 4; cz++)
                if (!world.chunkLoaded(cx, cz)) return OptionalDouble.empty();
        return world.findHighestCollisionTop(x - width, World.getMinWorldY(), z - width,
                x + width, World.getMaxWorldY() + 1.0, z + width);
    }

    /** Unknown columns and dimensions without a ceiling never establish above-roof status. */
    public static boolean isAboveRoof(World world, Vector3d feet) {
        OptionalDouble top = roofTop(world, feet.x, feet.z);
        return top.isPresent() && feet.y >= top.getAsDouble();
    }

    /** Whether a loaded collision face lies strictly above the target cell's feet. */
    public static boolean hasRoofAbove(World world, Vector3d target) {
        OptionalDouble top = roofTop(world, target.x, target.z);
        return top.isPresent() && top.getAsDouble() > target.y;
    }

    /** Verified roof landing footprint above the target, or null for unknown/unsafe geometry.
     * Uses shared support, clearance and environmental hazard landing checks.
     */
    public static Vector3d projectLandingTarget(World world, Vector3d target) {
        if (!hasRoofAbove(world, target)) return null;
        return VanillaGlideMode.landingSite(world, target.x, target.z, World.getMaxWorldY() + 1.0);
    }
}
