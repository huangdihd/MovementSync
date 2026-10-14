package xin.bbtt.tasks;

import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3d;
import org.joml.Vector3i;
import xin.bbtt.world.World;

/** Resolves vertical motion against exact block-state collision-box tops. */
public final class VerticalCollisionResolver {
    private VerticalCollisionResolver() {}

    public static double resolveDownwardY(
            World world, Vector3d currentFeet, double halfWidth, double displacementY) {
        return resolveDownwardY(world, currentFeet, halfWidth, displacementY, Set.of());
    }

    /** Powder snow's falling shape belongs to the entity context, not the static registry. */
    public static double resolveDownwardY(
            World world, Vector3d currentFeet, double halfWidth, double displacementY,
            double fallDistance) {
        double resolvedY = resolveDownwardY(world, currentFeet, halfWidth, displacementY);
        if (displacementY >= 0 || fallDistance <= 2.5) return resolvedY;

        for (int y = (int) Math.floor(currentFeet.y); y >= (int) Math.floor(resolvedY); y--) {
            double top = y + (double) 0.9F;
            if (top > currentFeet.y + 1.0e-7 || top < resolvedY) continue;
            for (int x = (int) Math.floor(currentFeet.x - halfWidth);
                    x < Math.ceil(currentFeet.x + halfWidth); x++) {
                for (int z = (int) Math.floor(currentFeet.z - halfWidth);
                        z < Math.ceil(currentFeet.z + halfWidth); z++) {
                    String name = world.getBlockStateAt(new Vector3d(x, y, z)).blockName();
                    if (name.equals("powder_snow") || name.equals("minecraft:powder_snow")) return top;
                }
            }
        }
        return resolvedY;
    }

    public static double resolveDownwardY(
            World world,
            Vector3d currentFeet,
            double halfWidth,
            double displacementY,
            Set<Vector3i> excludedBlocks) {
        double desiredY = currentFeet.y + displacementY;
        if (displacementY >= 0) return desiredY;

        OptionalDouble support = world.findHighestCollisionTopExcluding(
            currentFeet.x - halfWidth, desiredY, currentFeet.z - halfWidth,
            currentFeet.x + halfWidth, currentFeet.y, currentFeet.z + halfWidth,
            excludedBlocks);
        return support.isPresent() ? support.getAsDouble() : desiredY;
    }
}
