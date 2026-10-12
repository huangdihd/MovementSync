package xin.bbtt.pathfinding;

import java.util.HashSet;
import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3i;
import xin.bbtt.world.World;

/** Read-only collision projection of the cells cleared by an incoming DIG edge. */
final class ExcludedBlockCollisionView extends World {
    private final World world;
    private final Set<Vector3i> excludedBlocks;

    ExcludedBlockCollisionView(World world, Set<Vector3i> excludedBlocks) {
        this.world = world;
        this.excludedBlocks = new HashSet<>();
        for (Vector3i block : excludedBlocks) this.excludedBlocks.add(new Vector3i(block));
    }

    @Override
    public boolean chunkLoaded(int chunkX, int chunkZ) {
        return world.chunkLoaded(chunkX, chunkZ);
    }

    @Override
    public boolean isBoxCollidingExcluding(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ, Set<Vector3i> exclusions) {
        return world.isBoxCollidingExcluding(
                minX, minY, minZ, maxX, maxY, maxZ, combined(exclusions));
    }

    @Override
    public OptionalDouble findHighestCollisionTopExcluding(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ, Set<Vector3i> exclusions) {
        return world.findHighestCollisionTopExcluding(
                minX, minY, minZ, maxX, maxY, maxZ, combined(exclusions));
    }

    private Set<Vector3i> combined(Set<Vector3i> exclusions) {
        if (exclusions.isEmpty()) return excludedBlocks;
        Set<Vector3i> combined = new HashSet<>(excludedBlocks);
        combined.addAll(exclusions);
        return combined;
    }
}
