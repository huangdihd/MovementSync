package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundBlockUpdatePacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundForgetLevelChunkPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundLevelChunkWithLightPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundSectionBlocksUpdatePacket;
import xin.bbtt.MovementSync;

public class ChunkDataListener extends SessionAdapter {
    @Override
    public void packetReceived(Session session, Packet packet) {
        if (packet instanceof ClientboundLevelChunkWithLightPacket levelChunkWithLightPacket) {
            MovementSync.INSTANCE.getWorld().handleLevelChunkAndLightUpdate(levelChunkWithLightPacket);
        }
        if (packet instanceof ClientboundSectionBlocksUpdatePacket sectionBlocksUpdatePacket) {
            MovementSync.INSTANCE.getWorld().handleSectionBlocksUpdatePacket(sectionBlocksUpdatePacket);
            MovementSync.INSTANCE.requestRepath();
        }
        if (packet instanceof ClientboundBlockUpdatePacket blockUpdatePacket) {
            MovementSync.INSTANCE.getWorld().handleBlockUpdatePacket(blockUpdatePacket);
            MovementSync.INSTANCE.requestRepath();
        }
        if (packet instanceof ClientboundForgetLevelChunkPacket forgetLevelChunkPacket) {
            MovementSync.INSTANCE.getWorld().handleForgetLevelChunkPacket(forgetLevelChunkPacket);
            MovementSync.INSTANCE.requestRepath();
        }
    }
}