package xin.bbtt.listeners;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import net.kyori.adventure.key.Key;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.GameMode;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerSpawnInfo;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundRespawnPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundSetChunkCacheRadiusPacket;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;

final class WorldDataLifecycleTest {
    private final MovementSync previous = MovementSync.INSTANCE;
    private final MovementSync sync = new MovementSync();

    @AfterEach
    void restore() {
        MovementSync.INSTANCE = previous;
    }

    @Test
    void sameDimensionRespawnRetainsCacheButDimensionChangeClearsIt() {
        LoginPacketListener listener = new LoginPacketListener();
        listener.packetReceived(null, respawn(0, "minecraft:overworld"));
        sync.getWorld().handleSetChunkCacheRadius(new ClientboundSetChunkCacheRadiusPacket(12));

        listener.packetReceived(null, respawn(0, "minecraft:overworld"));
        assertEquals(12, sync.getWorld().getChunkCacheRadius().orElseThrow());

        listener.packetReceived(null, respawn(1, "minecraft:the_nether"));
        assertTrue(sync.getWorld().getChunkCacheRadius().isEmpty());
    }

    @Test
    void chunkListenerRoutesCacheMetadataPackets() {
        ChunkDataListener listener = new ChunkDataListener();
        listener.packetReceived(null, new ClientboundSetChunkCacheRadiusPacket(9));
        assertEquals(9, sync.getWorld().getChunkCacheRadius().orElseThrow());
    }

    @Test
    void disconnectClearsWorldData() {
        sync.getWorld().handleSetChunkCacheRadius(new ClientboundSetChunkCacheRadiusPacket(12));
        new ChunkDataListener().disconnected(null);
        assertTrue(sync.getWorld().getChunkCacheRadius().isEmpty());
    }

    private static ClientboundRespawnPacket respawn(int dimension, String worldName) {
        PlayerSpawnInfo spawn = new PlayerSpawnInfo(
                dimension, Key.key(worldName), 0, GameMode.SURVIVAL, null,
                false, false, null, 0, 63);
        return new ClientboundRespawnPacket(spawn, false, false);
    }
}
