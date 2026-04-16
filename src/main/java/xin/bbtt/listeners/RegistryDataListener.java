package xin.bbtt.listeners;

import lombok.Getter;
import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.configuration.clientbound.ClientboundRegistryDataPacket;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.Map;

public class RegistryDataListener extends SessionAdapter {
    @Getter
    private static int biomeRegistrySize = -1;
    
    @Getter
    private static final Map<String, Integer> enchantmentIds = new HashMap<>();
    
    @Getter
    private static final Map<Integer, String> networkIdToEnchantmentName = new HashMap<>();

    private static final Logger log = LoggerFactory.getLogger(RegistryDataListener.class.getSimpleName());

    @Override
    public void packetReceived(Session session, Packet packet) {
        if (!(packet instanceof ClientboundRegistryDataPacket registryPacket)) return;
        String registryName = registryPacket.getRegistry().asString();
        
        if (registryName.equals("minecraft:worldgen/biome")) {
            biomeRegistrySize = registryPacket.getEntries().size();
            log.debug("Got biomeRegistrySize: {}", biomeRegistrySize);
        } else if (registryName.equals("minecraft:enchantment")) {
            int index = 0;
            for (var entry : registryPacket.getEntries()) {
                String name = entry.getId().asString();
                enchantmentIds.put(name, index);
                networkIdToEnchantmentName.put(index, name);
                index++;
            }
            log.debug("Loaded {} enchantments from registry", enchantmentIds.size());
        }
    }
}
