package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.DisconnectedEvent;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.*;
import xin.bbtt.MovementSync;

public class ChunkDataListener extends SessionAdapter {
    @Override
    public void packetReceived(Session session, Packet packet) {
        if (packet instanceof ClientboundLevelChunkWithLightPacket levelChunkWithLightPacket) {
            MovementSync.INSTANCE.getWorld().handleLevelChunkAndLightUpdate(levelChunkWithLightPacket);
        }
        if (packet instanceof ClientboundSectionBlocksUpdatePacket sectionBlocksUpdatePacket) {
            MovementSync.INSTANCE.getWorld().handleSectionBlocksUpdatePacket(sectionBlocksUpdatePacket);
            for (var entry : sectionBlocksUpdatePacket.getEntries()) {
                MovementSync.INSTANCE.requestRepath(
                        entry.getPosition().getX(), entry.getPosition().getY(), entry.getPosition().getZ());
            }
        }
        if (packet instanceof ClientboundBlockUpdatePacket blockUpdatePacket) {
            MovementSync.INSTANCE.getWorld().handleBlockUpdatePacket(blockUpdatePacket);
            var changed = blockUpdatePacket.getEntry().getPosition();
            MovementSync.INSTANCE.requestRepath(changed.getX(), changed.getY(), changed.getZ());
        }
        if (packet instanceof ClientboundForgetLevelChunkPacket forgetLevelChunkPacket) {
            // Unloading a distant chunk doesn't invalidate the path the bot is
            // currently walking, so don't repath — doing so on every unload as
            // the bot travels made the route jitter and back up a little.
            MovementSync.INSTANCE.getWorld().handleForgetLevelChunkPacket(forgetLevelChunkPacket);
        }
        if (packet instanceof ClientboundLightUpdatePacket lightUpdatePacket) {
            MovementSync.INSTANCE.getWorld().handleLightUpdate(lightUpdatePacket);
        }
        if (packet instanceof ClientboundBlockEntityDataPacket blockEntityDataPacket) {
            MovementSync.INSTANCE.getWorld().handleBlockEntityData(blockEntityDataPacket);
        }
        if (packet instanceof ClientboundChunksBiomesPacket chunksBiomesPacket) {
            MovementSync.INSTANCE.getWorld().handleChunksBiomes(chunksBiomesPacket);
        }
        if (packet instanceof ClientboundSetChunkCacheCenterPacket cacheCenterPacket) {
            MovementSync.INSTANCE.getWorld().handleSetChunkCacheCenter(cacheCenterPacket);
        }
        if (packet instanceof ClientboundSetChunkCacheRadiusPacket cacheRadiusPacket) {
            MovementSync.INSTANCE.getWorld().handleSetChunkCacheRadius(cacheRadiusPacket);
        }
        if (packet instanceof ClientboundSetSimulationDistancePacket simulationDistancePacket) {
            MovementSync.INSTANCE.getWorld().handleSetSimulationDistance(simulationDistancePacket);
        }
        if (packet instanceof ClientboundSetDefaultSpawnPositionPacket defaultSpawnPacket) {
            MovementSync.INSTANCE.getWorld().handleSetDefaultSpawnPosition(defaultSpawnPacket);
        }
    }

    @Override
    public void disconnected(DisconnectedEvent event) {
        if (MovementSync.INSTANCE != null) MovementSync.INSTANCE.getWorld().clear();
    }
}