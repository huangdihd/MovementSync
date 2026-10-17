package xin.bbtt.movements;

import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class LandingMemoryTest {
    private static final Vector3d POSITION = new Vector3d(.5, 20, .5);

    private static final class Island extends World {
        boolean loaded = true;
        double centreX = 24.5;
        double centreZ = 8.5;

        @Override public boolean chunkLoaded(int x, int z) { return loaded; }

        @Override public OptionalDouble findHighestCollisionTopExcluding(double minX, double minY,
                double minZ, double maxX, double maxY, double maxZ, Set<Vector3i> excluded) {
            return loaded && minX >= centreX - 2.5 && maxX <= centreX + 2.5
                    && minZ >= centreZ - 2.5 && maxZ <= centreZ + 2.5
                    && minY <= 0 && maxY >= 0
                    ? OptionalDouble.of(0) : OptionalDouble.empty();
        }

        @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) {
            return false;
        }
    }

    @Test void observesNearbyGroundEvenWhenUnderfootIsVoid() {
        var memory = new LandingMemory();
        memory.observe(new Island(), POSITION);
        assertEquals(new Vector3d(24.5, 0, 8.5), memory.nearest(POSITION));
    }

    @Test void observesUnderfootAsWellAsNearbyChunks() {
        var world = new Island();
        world.centreX = POSITION.x;
        world.centreZ = POSITION.z;
        var memory = new LandingMemory();
        memory.observe(world, POSITION);
        assertEquals(new Vector3d(.5, 0, .5), memory.nearest(POSITION));
    }

    @Test void rotatesOffsetsToDiscoverOffCentreGround() {
        var world = new Island();
        world.centreX = 25.5;
        var memory = new LandingMemory();
        memory.observe(world, POSITION);
        assertNull(memory.nearest(POSITION));
        memory.observe(world, POSITION);
        assertEquals(new Vector3d(25.5, 0, 8.5), memory.nearest(POSITION));
    }

    @Test void unloadedTerrainRetainsPreviouslyObservedNearestSite() {
        var world = new Island();
        var memory = new LandingMemory();
        memory.observe(world, POSITION);
        world.loaded = false;
        memory.observe(world, POSITION);
        assertEquals(new Vector3d(24.5, 0, 8.5), memory.nearest(POSITION));
        var freshMemory = new LandingMemory();
        freshMemory.observe(world, POSITION);
        assertNull(freshMemory.nearest(POSITION));
    }

    @Test void longFlightRetainsDepartureTouchdownAndSpatialRoute() {
        var memory = new LandingMemory();
        var world = new Island();
        world.centreX = .5;
        world.centreZ = .5;
        memory.takeoff(world, POSITION);
        var departure = new Vector3d(.5, 0, .5);
        world.centreX = -200.5;
        var touchdown = new Vector3d(-200.5, 0, .5);
        memory.landed(world, touchdown);
        for (int i = 0; i < 600; i++) {
            world.centreX = i * 10 + .5;
            memory.observe(world, new Vector3d(world.centreX, 20, .5));
        }
        assertEquals(departure, memory.nearest(departure));
        assertEquals(touchdown, memory.nearest(touchdown));
        for (int x = 256; x < 5900; x += 128) {
            var point = new Vector3d(x + .5, 0, .5);
            assertTrue(memory.nearest(point).distance(point) < 128, "route coverage at " + x);
        }
        // Overflow must not evict either independently pinned site.
        for (int i = 600; i < 1600; i++) {
            world.centreX = i * 10 + .5;
            memory.observe(world, new Vector3d(world.centreX, 20, .5));
        }
        world.loaded = false;
        memory.takeoff(world, new Vector3d(20000, 20, 0));
        assertEquals(departure, memory.nearest(departure));
        assertEquals(touchdown, memory.nearest(touchdown));
        memory.clear();
        assertNull(memory.nearest(departure));
    }

    @Test void dimensionChangeClearsSitesButSameDimensionRetainsThem() {
        var memory = new LandingMemory();
        memory.dimension("minecraft:overworld");
        memory.observe(new Island(), POSITION);
        memory.dimension("minecraft:overworld");
        assertNotNull(memory.nearest(POSITION));
        memory.dimension("minecraft:the_nether");
        assertNull(memory.nearest(POSITION));
        // Clearing also restarts the centre-first sampling cycle.
        memory.observe(new Island(), POSITION);
        assertEquals(new Vector3d(24.5, 0, 8.5), memory.nearest(POSITION));
    }
}
