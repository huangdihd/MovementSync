package xin.bbtt.physics;

import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class LandingSearchReachableTest {
    private static final class Terrain extends World {
        double start = -100;
        double height;
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public Set<ChunkPosition> loadedChunks() {
            return Set.of(new ChunkPosition(0, 0), new ChunkPosition(0, 1), new ChunkPosition(0, 2));
        }
        @Override public OptionalDouble findHighestCollisionTopExcluding(double minX, double minY,
                double minZ, double maxX, double maxY, double maxZ, Set<Vector3i> excluded) {
            return minZ >= start && minY <= height && maxY >= height
                    ? OptionalDouble.of(height) : OptionalDouble.empty();
        }
    }

    private static PhysicsState state(double y) {
        return new PhysicsState(new Vector3d(.5, y, .5), new Vector3d(0, -.1, .8), false);
    }

    @Test void rejectedDirectBelowContinuesOrdinaryNearestFirstSamples() {
        Terrain world = new Terrain();
        var target = new LandingSearch().selectReachable(world, state(100), new int[0], false,
                MovementParameters.DEFAULT, site -> site.z >= 24);
        assertEquals(new Vector3d(.5, 0, 24.5), target);
    }

    @Test void rejectingEverySiteAlsoRejectsPoweredNearestFallbackAndPreviousTarget() {
        Terrain world = new Terrain();
        LandingSearch search = new LandingSearch();
        assertNotNull(search.selectReachable(world, state(100), new int[0], false,
                MovementParameters.DEFAULT, site -> true));
        assertNull(search.selectReachable(world, state(100), new int[]{1}, false,
                MovementParameters.DEFAULT, site -> false));
    }

    @Test void poweredFallbackSkipsRejectedNearestSite() {
        Terrain world = new Terrain();
        world.start = 20;
        var target = new LandingSearch().selectReachable(world, state(.1), new int[0], true,
                MovementParameters.DEFAULT, site -> site.z >= 40);
        assertNotNull(target);
        assertTrue(target.z >= 40);
    }

    @Test void corridorNeverSearchesAboveCurrentAltitude() {
        Terrain world = new Terrain();
        world.height = 80;
        assertNull(new LandingSearch().selectReachable(world, state(40), new int[]{1}, false,
                MovementParameters.DEFAULT, site -> true));
    }
}
