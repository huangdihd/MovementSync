package xin.bbtt.world;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import org.junit.jupiter.api.Test;

class WorldTest {

    @Test
    @SuppressWarnings("unchecked")
    void collisionQueriesUseCurrentSectionsAtNegativeCoordinates() throws Exception {
        World world = new World();
        Field field = World.class.getDeclaredField("chunks");
        field.setAccessible(true);
        Map<Integer, Map<Integer, Map<Integer,
                org.geysermc.mcprotocollib.protocol.data.game.chunk.ChunkSection>>> chunks =
                (Map<Integer, Map<Integer, Map<Integer,
                        org.geysermc.mcprotocollib.protocol.data.game.chunk.ChunkSection>>>) field.get(world);
        var section = new org.geysermc.mcprotocollib.protocol.data.game.chunk.ChunkSection(
                xin.bbtt.Block.BlockStateParser.getBlockStateRegistrySize(), 0, 64, 0);
        section.setBlock(15, 15, 15, 1);
        chunks.put(-1, new ConcurrentHashMap<>(Map.of(-1,
                new ConcurrentHashMap<>(Map.of(-1, section)))));
        assertEquals(1, world.getBlockAt(new org.joml.Vector3d(-0.5, -0.5, -0.5)));
        assertTrue(world.isBoxColliding(-0.9, -0.9, -0.9, -0.1, -0.1, -0.1));
        assertEquals(0, world.findHighestCollisionTop(-0.9, -0.1, -0.9, -0.1, 0.1, -0.1).orElseThrow());
        assertFalse(world.isBoxCollidingExcluding(-0.9, -0.9, -0.9, -0.1, -0.1, -0.1,
                Set.of(new org.joml.Vector3i(-1, -1, -1))));
        section.setBlock(15, 15, 15, 0);
        assertFalse(world.isBoxColliding(-0.9, -0.9, -0.9, -0.1, -0.1, -0.1));
        assertTrue(world.findHighestCollisionTop(-0.9, -0.1, -0.9, -0.1, 0.1, -0.1).isEmpty());
    }

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
