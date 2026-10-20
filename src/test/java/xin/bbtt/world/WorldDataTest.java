package xin.bbtt.world;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import java.lang.reflect.Field;
import java.util.BitSet;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalInt;
import java.util.concurrent.CompletableFuture;
import net.kyori.adventure.key.Key;
import org.cloudburstmc.math.vector.Vector3i;
import org.cloudburstmc.nbt.NbtMap;
import org.geysermc.mcprotocollib.protocol.codec.MinecraftTypes;
import org.geysermc.mcprotocollib.protocol.data.game.chunk.ChunkBiomeData;
import org.geysermc.mcprotocollib.protocol.data.game.chunk.ChunkSection;
import org.geysermc.mcprotocollib.protocol.data.game.chunk.DataPalette;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.GlobalPos;
import org.geysermc.mcprotocollib.protocol.data.game.level.HeightmapTypes;
import org.geysermc.mcprotocollib.protocol.data.game.level.LightUpdateData;
import org.geysermc.mcprotocollib.protocol.data.game.level.block.BlockChangeEntry;
import org.geysermc.mcprotocollib.protocol.data.game.level.block.BlockEntityInfo;
import org.geysermc.mcprotocollib.protocol.data.game.level.block.BlockEntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundBlockEntityDataPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundBlockUpdatePacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundChunksBiomesPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundForgetLevelChunkPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundLevelChunkWithLightPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundLightUpdatePacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundSetChunkCacheCenterPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundSetChunkCacheRadiusPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundSetDefaultSpawnPositionPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundSetSimulationDistancePacket;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.listeners.RegistryDataListener;

final class WorldDataTest {
    private static final int CHUNK_X = 1;
    private static final int CHUNK_Z = -2;
    private static final int BLOCK_X = 18;
    private static final int BLOCK_Z = -29;
    private static final int SECTION_COUNT = 24;
    private static final int BIOME_REGISTRY_SIZE = 16;

    @BeforeEach
    void configureRegistry() throws Exception {
        setDimension(-64, 384);
        setBiomeRegistrySize(BIOME_REGISTRY_SIZE);
        setBiomeNames(java.util.stream.IntStream.range(0, BIOME_REGISTRY_SIZE)
                .mapToObj(i -> "example:biome_" + i).toList());
    }

    @AfterEach
    void restoreRegistry() throws Exception {
        setDimension(-64, 384);
        setBiomeRegistrySize(-1);
        setBiomeNames(List.of());
    }

    @Test
    void chunkPacketExposesLightValuesUnknownSectionsAndWorldBounds() {
        byte[] pointLight = new byte[2048];
        setLight(pointLight, BLOCK_X, 5, BLOCK_Z, 12);

        World world = new World();
        world.handleLevelChunkAndLightUpdate(chunkPacket(
                bits(0, 1, 5, 24, 25), new BitSet(),
                List.of(uniformLight(9), uniformLight(3), pointLight,
                        uniformLight(14), uniformLight(8)),
                bits(5), List.of(), Map.of(), new BlockEntityInfo[0]));

        assertEquals(OptionalInt.of(12), world.getSkyLight(BLOCK_X, 5, BLOCK_Z));
        assertEquals(OptionalInt.of(3), world.getSkyLight(BLOCK_X, -64, BLOCK_Z));
        assertEquals(OptionalInt.of(14), world.getSkyLight(BLOCK_X, 319, BLOCK_Z));
        assertEquals(OptionalInt.of(0), world.getBlockLight(BLOCK_X, 5, BLOCK_Z),
                "an explicitly empty light section is known darkness");
        assertTrue(world.getSkyLight(BLOCK_X, 20, BLOCK_Z).isEmpty(),
                "an omitted light section remains unknown");
        assertTrue(world.getSkyLight(BLOCK_X + 16, 5, BLOCK_Z).isEmpty(),
                "an unloaded chunk has unknown light");
        assertTrue(world.getSkyLight(BLOCK_X, -65, BLOCK_Z).isEmpty(),
                "the protocol's lower padding light section is outside the world");
        assertTrue(world.getSkyLight(BLOCK_X, 320, BLOCK_Z).isEmpty(),
                "the protocol's upper padding light section is outside the world");
    }

    @Test
    void incrementalLightPacketReplacesAndThenEmptiesASection() {
        World world = loadEmptyChunk();
        world.handleLightUpdate(new ClientboundLightUpdatePacket(
                CHUNK_X, CHUNK_Z, bits(5, 6), new BitSet(), new BitSet(), new BitSet(),
                List.of(uniformLight(7), uniformLight(11)), List.of()));
        assertEquals(OptionalInt.of(7), world.getSkyLight(BLOCK_X, 5, BLOCK_Z));
        assertEquals(OptionalInt.of(11), world.getSkyLight(BLOCK_X, 20, BLOCK_Z));

        world.handleLightUpdate(new ClientboundLightUpdatePacket(
                CHUNK_X, CHUNK_Z, new BitSet(), new BitSet(), bits(5), new BitSet(),
                List.of(), List.of()));
        assertEquals(OptionalInt.of(0), world.getSkyLight(BLOCK_X, 5, BLOCK_Z));
        assertEquals(OptionalInt.of(11), world.getSkyLight(BLOCK_X, 20, BLOCK_Z),
                "a section omitted by both masks must keep its previous value");
    }

    @Test
    void repeatedFullChunkPacketPreservesLightSectionsOmittedByBothMasks() {
        World world = new World();
        world.handleLevelChunkAndLightUpdate(chunkPacket(
                bits(5), new BitSet(), List.of(uniformLight(7)),
                new BitSet(), List.of(), Map.of(), new BlockEntityInfo[0]));
        assertEquals(OptionalInt.of(7), world.getSkyLight(BLOCK_X, 5, BLOCK_Z));

        world.handleLevelChunkAndLightUpdate(chunkPacket(
                new BitSet(), new BitSet(), List.of(),
                new BitSet(), List.of(), Map.of(), new BlockEntityInfo[0]));

        assertEquals(OptionalInt.of(7), world.getSkyLight(BLOCK_X, 5, BLOCK_Z));
    }

    @Test
    void blockUpdatesRefreshCachedHeightmapColumns() {
        Map<HeightmapTypes, long[]> heightmaps = Map.of(
                HeightmapTypes.MOTION_BLOCKING, packedHeightmap(BLOCK_X, BLOCK_Z, 73));
        World world = new World();
        world.handleLevelChunkAndLightUpdate(chunkPacket(
                new BitSet(), new BitSet(), List.of(), new BitSet(), List.of(),
                heightmaps, new BlockEntityInfo[0]));

        world.handleBlockUpdatePacket(new ClientboundBlockUpdatePacket(
                new BlockChangeEntry(Vector3i.from(BLOCK_X, 80, BLOCK_Z), 1)));
        assertEquals(OptionalInt.of(80),
                world.getHeight(HeightmapTypes.MOTION_BLOCKING, BLOCK_X, BLOCK_Z));

        world.handleBlockUpdatePacket(new ClientboundBlockUpdatePacket(
                new BlockChangeEntry(Vector3i.from(BLOCK_X, 80, BLOCK_Z), 0)));
        assertEquals(OptionalInt.of(-65),
                world.getHeight(HeightmapTypes.MOTION_BLOCKING, BLOCK_X, BLOCK_Z));
    }

    @Test
    void compoundReadsSeeCompleteLightPacketStatesDuringConcurrentUpdates() {
        World world = loadEmptyChunk();
        ClientboundLightUpdatePacket low = lightPair(2, 4);
        ClientboundLightUpdatePacket high = lightPair(11, 13);
        world.handleLightUpdate(low);

        CompletableFuture<Void> writer = CompletableFuture.runAsync(() -> {
            for (int i = 0; i < 2_000; i++) {
                world.handleLightUpdate((i & 1) == 0 ? high : low);
            }
        });
        for (int i = 0; i < 2_000; i++) {
            assertTrue(world.readTerrain(() -> {
                int sky = world.getSkyLight(BLOCK_X, 5, BLOCK_Z).orElseThrow();
                int block = world.getBlockLight(BLOCK_X, 5, BLOCK_Z).orElseThrow();
                return (sky == 2 && block == 4) || (sky == 11 && block == 13);
            }), "a compound read must not mix two light packet states");
        }
        writer.join();
    }

    private static ClientboundLightUpdatePacket lightPair(int sky, int block) {
        return new ClientboundLightUpdatePacket(
                CHUNK_X, CHUNK_Z, bits(5), bits(5), new BitSet(), new BitSet(),
                List.of(uniformLight(sky)), List.of(uniformLight(block)));
    }

    @Test
    void blockEntitiesLoadUpdateRemoveAndDisappearOnUnload() {
        NbtMap original = NbtMap.builder().putString("CustomName", "first").build();
        BlockEntityInfo info = new BlockEntityInfo(2, 5, 3, BlockEntityType.CHEST, original);
        World world = new World();
        world.handleLevelChunkAndLightUpdate(chunkPacket(
                new BitSet(), new BitSet(), List.of(), new BitSet(), List.of(),
                Map.of(), new BlockEntityInfo[]{info}));

        World.BlockEntitySnapshot loaded = world.getBlockEntityAt(BLOCK_X, 5, BLOCK_Z).orElseThrow();
        assertEquals(BlockEntityType.CHEST, loaded.type());
        assertEquals("first", loaded.nbt().getString("CustomName"));
        assertEquals(List.of(loaded), world.getBlockEntitiesInChunk(CHUNK_X, CHUNK_Z));

        NbtMap replacement = NbtMap.builder().putInt("Levels", 4).build();
        world.handleBlockEntityData(new ClientboundBlockEntityDataPacket(
                Vector3i.from(BLOCK_X, 5, BLOCK_Z), BlockEntityType.CHEST, replacement));
        World.BlockEntitySnapshot updated = world.getBlockEntityAt(BLOCK_X, 5, BLOCK_Z).orElseThrow();
        assertEquals(BlockEntityType.CHEST, updated.type());
        assertEquals(4, updated.nbt().getInt("Levels"));

        world.handleBlockUpdatePacket(new ClientboundBlockUpdatePacket(
                new BlockChangeEntry(Vector3i.from(BLOCK_X, 5, BLOCK_Z), 1)));
        assertTrue(world.getBlockEntityAt(BLOCK_X, 5, BLOCK_Z).isEmpty(),
                "changing the block kind removes stale block-entity data");

        world.handleBlockEntityData(new ClientboundBlockEntityDataPacket(
                Vector3i.from(BLOCK_X, 5, BLOCK_Z), BlockEntityType.CHEST, original));
        assertTrue(world.getBlockEntityAt(BLOCK_X, 5, BLOCK_Z).isPresent(),
                "a later server update can create the replacement block entity");
        world.handleForgetLevelChunkPacket(new ClientboundForgetLevelChunkPacket(CHUNK_X, CHUNK_Z));
        assertTrue(world.getBlockEntityAt(BLOCK_X, 5, BLOCK_Z).isEmpty());
        assertTrue(world.getBlockEntitiesInChunk(CHUNK_X, CHUNK_Z).isEmpty());
        assertFalse(world.chunkLoaded(CHUNK_X, CHUNK_Z));
    }

    @Test
    void chunkPacketExposesAbsoluteHeightmapValuesAndDefensivelyCopiesThem() {
        long[] heights = packedHeightmap(BLOCK_X, BLOCK_Z, 73);
        Map<HeightmapTypes, long[]> heightmaps = new EnumMap<>(HeightmapTypes.class);
        heightmaps.put(HeightmapTypes.MOTION_BLOCKING, heights);
        World world = new World();
        world.handleLevelChunkAndLightUpdate(chunkPacket(
                new BitSet(), new BitSet(), List.of(), new BitSet(), List.of(),
                heightmaps, new BlockEntityInfo[0]));
        heights[heightCell(BLOCK_X, BLOCK_Z)] = 0;

        assertEquals(OptionalInt.of(73),
                world.getHeight(HeightmapTypes.MOTION_BLOCKING, BLOCK_X, BLOCK_Z));
        assertTrue(world.getHeight(HeightmapTypes.WORLD_SURFACE, BLOCK_X, BLOCK_Z).isEmpty());
        assertTrue(world.getHeight(
                HeightmapTypes.MOTION_BLOCKING, BLOCK_X + 16, BLOCK_Z).isEmpty());
    }

    @Test
    void chunksBiomesPacketUpdatesQuartBiomesWithoutChangingBlocks() {
        World world = loadEmptyChunk();
        world.handleBlockUpdatePacket(new ClientboundBlockUpdatePacket(
                new BlockChangeEntry(Vector3i.from(BLOCK_X, 5, BLOCK_Z), 1)));
        assertEquals(OptionalInt.of(0), world.getBiomeId(BLOCK_X, 5, BLOCK_Z));

        world.handleChunksBiomes(new ClientboundChunksBiomesPacket(List.of(
                new ChunkBiomeData(CHUNK_X, CHUNK_Z, biomeSections(7)))));

        assertEquals(OptionalInt.of(7), world.getBiomeId(BLOCK_X, 5, BLOCK_Z));
        assertEquals("example:biome_7", world.getBiomeName(BLOCK_X, 5, BLOCK_Z).orElseThrow());
        assertEquals(1, world.getBlockAt(new org.joml.Vector3d(BLOCK_X, 5, BLOCK_Z)),
                "a biome-only packet must preserve block data");
        assertTrue(world.getBiomeId(BLOCK_X + 16, 5, BLOCK_Z).isEmpty());
    }

    @Test
    void cacheSimulationAndDefaultSpawnPacketsReplaceTheirLatestValues() {
        World world = new World();
        assertTrue(world.getChunkCacheCenter().isEmpty());
        assertTrue(world.getChunkCacheRadius().isEmpty());
        assertTrue(world.getSimulationDistance().isEmpty());
        assertTrue(world.getDefaultSpawn().isEmpty());

        GlobalPos spawn = new GlobalPos(Key.key("minecraft:overworld"), Vector3i.from(2, 70, -4));
        world.handleSetChunkCacheCenter(new ClientboundSetChunkCacheCenterPacket(-7, 11));
        world.handleSetChunkCacheRadius(new ClientboundSetChunkCacheRadiusPacket(12));
        world.handleSetSimulationDistance(new ClientboundSetSimulationDistancePacket(8));
        world.handleSetDefaultSpawnPosition(
                new ClientboundSetDefaultSpawnPositionPacket(spawn, 45.0f, -10.0f));
        assertEquals(new World.ChunkPosition(-7, 11), world.getChunkCacheCenter().orElseThrow());
        assertEquals(OptionalInt.of(12), world.getChunkCacheRadius());
        assertEquals(OptionalInt.of(8), world.getSimulationDistance());
        assertEquals(new World.DefaultSpawn(spawn, 45.0f, -10.0f),
                world.getDefaultSpawn().orElseThrow());

        world.handleSetChunkCacheCenter(new ClientboundSetChunkCacheCenterPacket(4, -3));
        world.handleSetChunkCacheRadius(new ClientboundSetChunkCacheRadiusPacket(5));
        world.handleSetSimulationDistance(new ClientboundSetSimulationDistancePacket(2));
        assertEquals(new World.ChunkPosition(4, -3), world.getChunkCacheCenter().orElseThrow());
        assertEquals(OptionalInt.of(5), world.getChunkCacheRadius());
        assertEquals(OptionalInt.of(2), world.getSimulationDistance());
    }

    @Test
    void clearRemovesChunksAndAllAssociatedWorldData() {
        Map<HeightmapTypes, long[]> heightmaps = Map.of(
                HeightmapTypes.MOTION_BLOCKING, packedHeightmap(BLOCK_X, BLOCK_Z, 73));
        BlockEntityInfo entity = new BlockEntityInfo(
                2, 5, 3, BlockEntityType.CHEST, NbtMap.EMPTY);
        World world = new World();
        world.handleLevelChunkAndLightUpdate(chunkPacket(
                bits(5), new BitSet(), List.of(uniformLight(6)), new BitSet(), List.of(),
                heightmaps, new BlockEntityInfo[]{entity}));
        world.handleSetChunkCacheCenter(new ClientboundSetChunkCacheCenterPacket(1, 2));
        world.handleSetChunkCacheRadius(new ClientboundSetChunkCacheRadiusPacket(10));
        world.handleSetSimulationDistance(new ClientboundSetSimulationDistancePacket(6));
        world.handleSetDefaultSpawnPosition(new ClientboundSetDefaultSpawnPositionPacket(
                new GlobalPos(Key.key("minecraft:overworld"), Vector3i.from(0, 64, 0)), 0, 0));

        assertTrue(world.getSkyLight(BLOCK_X, 5, BLOCK_Z).isPresent());
        assertTrue(world.getBlockEntityAt(BLOCK_X, 5, BLOCK_Z).isPresent());
        assertTrue(world.getHeight(HeightmapTypes.MOTION_BLOCKING, BLOCK_X, BLOCK_Z).isPresent());
        world.clear();

        assertTrue(world.loadedChunks().isEmpty());
        assertTrue(world.getSkyLight(BLOCK_X, 5, BLOCK_Z).isEmpty());
        assertTrue(world.getBlockEntityAt(BLOCK_X, 5, BLOCK_Z).isEmpty());
        assertTrue(world.getHeight(HeightmapTypes.MOTION_BLOCKING, BLOCK_X, BLOCK_Z).isEmpty());
        assertTrue(world.getBiomeId(BLOCK_X, 5, BLOCK_Z).isEmpty());
        assertTrue(world.getChunkCacheCenter().isEmpty());
        assertTrue(world.getChunkCacheRadius().isEmpty());
        assertTrue(world.getSimulationDistance().isEmpty());
        assertTrue(world.getDefaultSpawn().isEmpty());
    }

    private static World loadEmptyChunk() {
        World world = new World();
        world.handleLevelChunkAndLightUpdate(chunkPacket(
                new BitSet(), new BitSet(), List.of(), new BitSet(), List.of(),
                Map.of(), new BlockEntityInfo[0]));
        return world;
    }

    private static ClientboundLevelChunkWithLightPacket chunkPacket(
            BitSet skyMask, BitSet blockMask, List<byte[]> skyUpdates,
            BitSet emptyBlockMask, List<byte[]> blockUpdates,
            Map<HeightmapTypes, long[]> heightmaps, BlockEntityInfo[] blockEntities) {
        ByteBuf buffer = Unpooled.buffer();
        try {
            for (int i = 0; i < SECTION_COUNT; i++) {
                MinecraftTypes.writeChunkSection(buffer, new ChunkSection(
                        0, BlockStateParser.getBlockStateRegistrySize(), 0, BIOME_REGISTRY_SIZE));
            }
            byte[] chunkData = new byte[buffer.readableBytes()];
            buffer.readBytes(chunkData);
            LightUpdateData light = new LightUpdateData(
                    skyMask, blockMask, new BitSet(), emptyBlockMask, skyUpdates, blockUpdates);
            return new ClientboundLevelChunkWithLightPacket(
                    CHUNK_X, CHUNK_Z, chunkData, heightmaps, blockEntities, light);
        } finally {
            buffer.release();
        }
    }

    private static byte[] biomeSections(int biomeId) {
        ByteBuf buffer = Unpooled.buffer();
        try {
            for (int i = 0; i < SECTION_COUNT; i++) {
                MinecraftTypes.writeDataPalette(
                        buffer, DataPalette.createForBiome(biomeId, BIOME_REGISTRY_SIZE));
            }
            byte[] result = new byte[buffer.readableBytes()];
            buffer.readBytes(result);
            return result;
        } finally {
            buffer.release();
        }
    }

    private static byte[] uniformLight(int value) {
        byte packed = (byte) (value | (value << 4));
        byte[] data = new byte[2048];
        java.util.Arrays.fill(data, packed);
        return data;
    }

    private static void setLight(byte[] data, int x, int y, int z, int value) {
        int index = ((y & 15) << 8) | ((z & 15) << 4) | (x & 15);
        int old = data[index >> 1] & 0xff;
        int packed = (index & 1) == 0
                ? (old & 0xf0) | value
                : (old & 0x0f) | (value << 4);
        data[index >> 1] = (byte) packed;
    }

    private static long[] packedHeightmap(int blockX, int blockZ, int absoluteY) {
        int bitsPerValue = 9;
        int valuesPerLong = 64 / bitsPerValue;
        int index = ((blockZ & 15) << 4) | (blockX & 15);
        long[] values = new long[(256 + valuesPerLong - 1) / valuesPerLong];
        values[index / valuesPerLong] = (long) (absoluteY + 65)
                << ((index % valuesPerLong) * bitsPerValue);
        return values;
    }

    private static int heightCell(int blockX, int blockZ) {
        int index = ((blockZ & 15) << 4) | (blockX & 15);
        return index / (64 / 9);
    }

    private static BitSet bits(int... indexes) {
        BitSet result = new BitSet();
        for (int index : indexes) result.set(index);
        return result;
    }

    @SuppressWarnings("unchecked")
    private static void setDimension(int minY, int height) throws Exception {
        Field typesField = RegistryDataListener.class.getDeclaredField("dimensionTypes");
        typesField.setAccessible(true);
        List<NbtMap> types = (List<NbtMap>) typesField.get(null);
        types.clear();
        types.add(NbtMap.builder().putInt("min_y", minY).putInt("height", height).build());

        Field indexField = RegistryDataListener.class.getDeclaredField("currentDimensionIndex");
        indexField.setAccessible(true);
        indexField.setInt(null, 0);
    }

    private static void setBiomeRegistrySize(int size) throws Exception {
        Field field = RegistryDataListener.class.getDeclaredField("biomeRegistrySize");
        field.setAccessible(true);
        field.setInt(null, size);
    }

    private static void setBiomeNames(List<String> names) throws Exception {
        Field field = RegistryDataListener.class.getDeclaredField("biomeNames");
        field.setAccessible(true);
        field.set(null, names);
    }
}
