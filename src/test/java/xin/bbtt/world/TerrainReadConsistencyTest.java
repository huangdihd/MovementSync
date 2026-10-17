package xin.bbtt.world;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import org.junit.jupiter.api.Test;

final class TerrainReadConsistencyTest {
    @Test
    void clearCannotUnloadTerrainBetweenCompoundReads() throws Exception {
        World world = worldWithLoadedChunk();
        CountDownLatch firstRead = new CountDownLatch(1);
        CountDownLatch finishRead = new CountDownLatch(1);
        CountDownLatch writerStarted = new CountDownLatch(1);
        CompletableFuture<Boolean> reader = CompletableFuture.supplyAsync(() -> world.readTerrain(() -> {
            assertTrue(world.chunkLoaded(2, 3));
            firstRead.countDown();
            await(finishRead);
            // These nested read-lock acquisitions must remain safe even with a writer waiting.
            assertEquals(Set.of(new World.ChunkPosition(2, 3)), world.loadedChunks());
            return world.chunkLoaded(2, 3);
        }));
        CompletableFuture<Void> writer = null;
        try {
            assertTrue(firstRead.await(2, TimeUnit.SECONDS));
            writer = CompletableFuture.runAsync(() -> {
                writerStarted.countDown();
                // Real bulk unload, without packet event dispatch or a Bot singleton.
                world.clear();
            });
            assertTrue(writerStarted.await(2, TimeUnit.SECONDS));
            CompletableFuture<Void> pendingWriter = writer;
            assertThrows(TimeoutException.class, () -> pendingWriter.get(100, TimeUnit.MILLISECONDS),
                "terrain writes must wait for the whole compound query");
        } finally {
            finishRead.countDown();
        }
        assertTrue(reader.get(2, TimeUnit.SECONDS));
        assertNotNull(writer);
        writer.get(2, TimeUnit.SECONDS);
        assertFalse(world.chunkLoaded(2, 3));
        assertTrue(world.loadedChunks().isEmpty());
    }

    @Test
    void throwingCompoundReadReleasesItsLockForOtherThreads() throws Exception {
        World world = worldWithLoadedChunk();
        IllegalStateException failure = new IllegalStateException("query failed");
        assertSame(failure, assertThrows(IllegalStateException.class, () -> world.readTerrain(() -> {
            assertTrue(world.chunkLoaded(2, 3));
            throw failure;
        })));

        CompletableFuture.runAsync(world::clear).get(2, TimeUnit.SECONDS);
        assertFalse(world.readTerrain(() -> world.chunkLoaded(2, 3)));
    }

    private static void await(CountDownLatch latch) {
        try {
            assertTrue(latch.await(5, TimeUnit.SECONDS), "compound query was not released");
        } catch (InterruptedException error) {
            Thread.currentThread().interrupt();
            throw new AssertionError("interrupted while waiting for compound query", error);
        }
    }

    @SuppressWarnings("unchecked")
    private static World worldWithLoadedChunk() throws Exception {
        World world = new World();
        Field chunksField = World.class.getDeclaredField("chunks");
        chunksField.setAccessible(true);
        Map<Integer, Map<Integer, Map<Integer, ?>>> chunks =
            (Map<Integer, Map<Integer, Map<Integer, ?>>>) chunksField.get(world);
        chunks.computeIfAbsent(2, ignored -> new ConcurrentHashMap<>())
            .put(3, new ConcurrentHashMap<>());
        return world;
    }
}
