package xin.bbtt.world;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import org.junit.jupiter.api.Test;

class WorldTest {

    @Test
    void defaultValueIsVanillaOverworld() {
        // Without dimension registry data, defaults to vanilla overworld.
        assertEquals(-64, World.getMinWorldY());
        assertEquals(319, World.getMaxWorldY());
    }

    @Test
    void withinFallbackBoundsAtMin() {
        assertTrue(World.isWithinWorldBounds(World.getMinWorldY()));
    }

    @Test
    void withinFallbackBoundsAtMax() {
        assertTrue(World.isWithinWorldBounds(World.getMaxWorldY()));
    }

    @Test
    void withinBoundsAtZero() {
        assertTrue(World.isWithinWorldBounds(0));
    }

    @Test
    void belowMinIsOutOfBounds() {
        assertFalse(World.isWithinWorldBounds(World.getMinWorldY() - 1));
    }

    @Test
    void aboveMaxIsOutOfBounds() {
        assertFalse(World.isWithinWorldBounds(World.getMaxWorldY() + 1));
    }

    @Test
    @SuppressWarnings("unchecked")
    void returnsImmutableSnapshotOfExactlyLoadedChunkCoordinates() throws Exception {
        World world = new World();
        Field chunksField = World.class.getDeclaredField("chunks");
        chunksField.setAccessible(true);
        Map<Integer, Map<Integer, Map<Integer, ?>>> chunks =
            (Map<Integer, Map<Integer, Map<Integer, ?>>>) chunksField.get(world);
        chunks.computeIfAbsent(-2, ignored -> new ConcurrentHashMap<>())
            .put(3, new ConcurrentHashMap<>());
        chunks.computeIfAbsent(4, ignored -> new ConcurrentHashMap<>())
            .put(-5, new ConcurrentHashMap<>());

        Set<World.ChunkPosition> snapshot = world.loadedChunks();

        assertEquals(Set.of(new World.ChunkPosition(-2, 3), new World.ChunkPosition(4, -5)), snapshot);
        assertThrows(UnsupportedOperationException.class, () -> snapshot.add(new World.ChunkPosition(0, 0)));
        chunks.get(-2).remove(3);
        assertEquals(2, snapshot.size(), "the returned set must not be a live view");
    }
}
