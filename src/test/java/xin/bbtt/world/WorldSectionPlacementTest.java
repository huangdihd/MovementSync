package xin.bbtt.world;

import static org.junit.jupiter.api.Assertions.assertEquals;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import org.cloudburstmc.nbt.NbtMap;
import org.geysermc.mcprotocollib.protocol.codec.MinecraftTypes;
import org.geysermc.mcprotocollib.protocol.data.game.chunk.ChunkSection;
import org.geysermc.mcprotocollib.protocol.data.game.level.LightUpdateData;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundLevelChunkWithLightPacket;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.listeners.RegistryDataListener;

final class WorldSectionPlacementTest {
    @AfterEach
    void resetDimensionRegistry() throws Exception {
        setDimension(-64, 384);
    }

    @Test
    void overworldSectionsStartAtNegativeFour() throws Exception {
        assertFirstAndLastSection(-64, 384, -4, 19);
    }

    @Test
    void netherSectionsStartAtZero() throws Exception {
        assertFirstAndLastSection(0, 256, 0, 15);
    }

    @Test
    void endSectionsStartAtZero() throws Exception {
        assertFirstAndLastSection(0, 256, 0, 15);
    }

    @Test
    void customHeightSectionsUseDimensionMinimum() throws Exception {
        assertFirstAndLastSection(-128, 512, -8, 23);
    }

    @SuppressWarnings("unchecked")
    private static void assertFirstAndLastSection(
            int minY, int height, int expectedFirst, int expectedLast) throws Exception {
        setDimension(minY, height);
        World world = new World();
        world.handleLevelChunkAndLightUpdate(chunkPacket(height / 16));

        Field chunksField = World.class.getDeclaredField("chunks");
        chunksField.setAccessible(true);
        Map<Integer, Map<Integer, Map<Integer, ChunkSection>>> chunks =
                (Map<Integer, Map<Integer, Map<Integer, ChunkSection>>>) chunksField.get(world);
        Map<Integer, ChunkSection> sections = chunks.get(2).get(3);
        assertEquals(expectedFirst, sections.keySet().stream().mapToInt(Integer::intValue).min().orElseThrow());
        assertEquals(expectedLast, sections.keySet().stream().mapToInt(Integer::intValue).max().orElseThrow());
    }

    private static ClientboundLevelChunkWithLightPacket chunkPacket(int sectionCount) {
        ByteBuf buffer = Unpooled.buffer();
        try {
            for (int i = 0; i < sectionCount; i++) {
                MinecraftTypes.writeChunkSection(buffer, new ChunkSection(
                        0, BlockStateParser.getBlockStateRegistrySize(), 0, 64));
            }
            byte[] chunkData = new byte[buffer.readableBytes()];
            buffer.readBytes(chunkData);
            LightUpdateData lightData = new LightUpdateData(
                    new java.util.BitSet(), new java.util.BitSet(),
                    new java.util.BitSet(), new java.util.BitSet(), List.of(), List.of());
            return new ClientboundLevelChunkWithLightPacket(
                    2, 3, chunkData, Map.of(),
                    new org.geysermc.mcprotocollib.protocol.data.game.level.block.BlockEntityInfo[0], lightData);
        } finally {
            buffer.release();
        }
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

        Field biomeSizeField = RegistryDataListener.class.getDeclaredField("biomeRegistrySize");
        biomeSizeField.setAccessible(true);
        biomeSizeField.setInt(null, 64);
    }
}
