package xin.bbtt.listeners;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.util.List;
import net.kyori.adventure.key.Key;
import org.geysermc.mcprotocollib.protocol.data.game.RegistryEntry;
import org.geysermc.mcprotocollib.protocol.packet.configuration.clientbound.ClientboundRegistryDataPacket;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

final class RegistryDataListenerTest {
    @AfterEach
    void restoreBiomeRegistry() throws Exception {
        setBiomeNames(List.of());
        setBiomeRegistrySize(-1);
    }

    @Test
    void biomeRegistryPublishesOrderedNamesAndReplacesOldMappings() throws Exception {
        RegistryDataListener listener = new RegistryDataListener();
        listener.packetReceived(null, new ClientboundRegistryDataPacket(
                Key.key("minecraft:worldgen/biome"), List.of(
                        new RegistryEntry(Key.key("minecraft:plains"), null),
                        new RegistryEntry(Key.key("example:custom"), null))));

        assertEquals(2, RegistryDataListener.getBiomeRegistrySize());
        assertEquals("minecraft:plains", RegistryDataListener.getBiomeName(0).orElseThrow());
        assertEquals("example:custom", RegistryDataListener.getBiomeName(1).orElseThrow());

        listener.packetReceived(null, new ClientboundRegistryDataPacket(
                Key.key("minecraft:worldgen/biome"), List.of(
                        new RegistryEntry(Key.key("minecraft:desert"), null))));

        assertEquals(1, RegistryDataListener.getBiomeRegistrySize());
        assertEquals("minecraft:desert", RegistryDataListener.getBiomeName(0).orElseThrow());
        assertTrue(RegistryDataListener.getBiomeName(1).isEmpty(),
                "a replacement registry must not retain stale network IDs");
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
