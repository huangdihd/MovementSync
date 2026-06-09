package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundLoginPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundRespawnPacket;
import xin.bbtt.MovementSync;

/**
 * Captures the current dimension index from login and respawn packets,
 * so {@code RegistryDataListener} can return the correct min/max world Y.
 */
public class LoginPacketListener extends SessionAdapter {

    @Override
    public void packetReceived(Session session, Packet packet) {
        if (packet instanceof ClientboundLoginPacket login) {
            int dim = login.getCommonPlayerSpawnInfo().getDimension();
            RegistryDataListener.setCurrentDimension(dim);
            MovementSync.getLogger().debug("Login: dimension index = {}", dim);
        } else if (packet instanceof ClientboundRespawnPacket respawn) {
            int dim = respawn.getCommonPlayerSpawnInfo().getDimension();
            RegistryDataListener.setCurrentDimension(dim);
            MovementSync.getLogger().debug("Respawn: dimension index = {}", dim);
        }
    }
}
