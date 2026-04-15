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
            MovementSync.Instance.getWorld().handleLevelChunkAndLightUpdate(levelChunkWithLightPacket);
        }
        if (packet instanceof ClientboundSectionBlocksUpdatePacket sectionBlocksUpdatePacket) MovementSync.Instance.getWorld().handleSectionBlocksUpdatePacket(sectionBlocksUpdatePacket);
        if (packet instanceof ClientboundBlockUpdatePacket blockUpdatePacket) MovementSync.Instance.getWorld().handleBlockUpdatePacket(blockUpdatePacket);
        if (packet instanceof ClientboundForgetLevelChunkPacket forgetLevelChunkPacket) MovementSync.Instance.getWorld().handleForgetLevelChunkPacket(forgetLevelChunkPacket);
    }
}