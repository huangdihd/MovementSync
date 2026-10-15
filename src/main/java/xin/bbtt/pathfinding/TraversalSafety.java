package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Navigation excludes traps even when their collision shape is empty. */
final class TraversalSafety {
    private static final double EPSILON = 1.0e-7;

    private TraversalSafety() {}

    static boolean isSafe(World world, Vector3d position) {
        double halfWidth = StandablePositionResolver.PLAYER_HALF_WIDTH;
        int minimumX = (int) Math.floor(position.x - halfWidth + EPSILON);
        int maximumX = (int) Math.floor(position.x + halfWidth - EPSILON);
        int minimumZ = (int) Math.floor(position.z - halfWidth + EPSILON);
        int maximumZ = (int) Math.floor(position.z + halfWidth - EPSILON);
        int maximumY = (int) Math.floor(position.y + StandablePositionResolver.PLAYER_HEIGHT - EPSILON);
        for (int x = minimumX; x <= maximumX; x++) {
            for (int z = minimumZ; z <= maximumZ; z++) {
                if (!world.chunkLoaded(x >> 4, z >> 4)) return false;
                for (int y = (int) Math.floor(position.y + EPSILON); y <= maximumY; y++) {
                    String name = world.getBlockStateAt(new Vector3d(x, y, z)).blockName();
                    if ("cobweb".equals(name) || "sweet_berry_bush".equals(name)
                            || "powder_snow".equals(name) || "lava".equals(name)) return false;
                }
            }
        }
        return true;
    }

    static boolean isSafeSegment(World world, Vector3d source, Vector3d target) {
        int samples = Math.max(1, (int) Math.ceil(source.distance(target) / 0.2));
        for (int sample = 0; sample <= samples; sample++) {
            Vector3d position = new Vector3d(source).lerp(target, (double) sample / samples);
            if (!isSafe(world, position)) return false;
        }
        return true;
    }
}
