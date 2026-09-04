package xin.bbtt.tasks;

import java.util.OptionalDouble;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Resolves vertical motion against exact block-state collision-box tops. */
final class VerticalCollisionResolver {
    private VerticalCollisionResolver() {}

    static double resolveDownwardY(
            World world, Vector3d currentFeet, double halfWidth, double displacementY) {
        double desiredY = currentFeet.y + displacementY;
        if (displacementY >= 0) return desiredY;

        OptionalDouble support = world.findHighestCollisionTop(
            currentFeet.x - halfWidth, desiredY, currentFeet.z - halfWidth,
            currentFeet.x + halfWidth, currentFeet.y, currentFeet.z + halfWidth);
        return support.isPresent() ? support.getAsDouble() : desiredY;
    }
}
