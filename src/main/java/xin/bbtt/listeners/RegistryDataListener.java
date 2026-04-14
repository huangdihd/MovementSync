package xin.bbtt.listeners;

import lombok.Getter;
import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.configuration.clientbound.ClientboundRegistryDataPacket;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RegistryDataListener extends SessionAdapter {
    @Getter
    private static int biomeRegistrySize = -1;
    private static final Logger log = LoggerFactory.getLogger(RegistryDataListener.class.getSimpleName());

    @Override
    public void packetReceived(Session session, Packet packet) {
        if (!(packet instanceof ClientboundRegistryDataPacket registryPacket)) return;
        if (!registryPacket.getRegistry().asString().equals("minecraft:worldgen/biome")) return;
        biomeRegistrySize = registryPacket.getEntries().size();
        log.debug("Got biomeRegistrySize: {}", biomeRegistrySize);
    }
}
