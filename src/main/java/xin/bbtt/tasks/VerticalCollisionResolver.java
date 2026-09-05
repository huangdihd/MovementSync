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
