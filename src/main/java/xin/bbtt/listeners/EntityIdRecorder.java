package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundLoginPacket;
import xin.bbtt.MovementSync;

public class EntityIdRecorder extends SessionAdapter {
    @Override
    public void packetReceived(Session session, Packet packet) {
        if (!(packet instanceof ClientboundLoginPacket loginPacket)) return;
        MovementSync.getLogger().info(xin.bbtt.mcbot.LangManager.get("movementsync.listener.entityid"), loginPacket.getEntityId());
        MovementSync.INSTANCE.entityId = loginPacket.getEntityId();
    }
}
