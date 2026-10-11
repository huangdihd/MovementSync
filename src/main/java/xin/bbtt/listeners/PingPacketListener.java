package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.common.clientbound.ClientboundPingPacket;
import org.geysermc.mcprotocollib.protocol.packet.common.serverbound.ServerboundPongPacket;

/**
 * Answers server pings like the vanilla client. The protocol library only
 * answers keep-alives; anti-cheats use ping/pong pairs to order movement
 * packets in time, and reject every move from a client that never replies.
 */
public class PingPacketListener extends SessionAdapter {
    @Override
    public void packetReceived(Session session, Packet packet) {
        if (!(packet instanceof ClientboundPingPacket pingPacket)) return;
        session.send(new ServerboundPongPacket(pingPacket.getId()));
    }
}
