package xin.bbtt.listeners;

import lombok.Getter;
import org.cloudburstmc.nbt.NbtMap;
import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.configuration.clientbound.ClientboundRegistryDataPacket;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RegistryDataListener extends SessionAdapter {
    @Getter
    private static int biomeRegistrySize = -1;
    
    @Getter
    private static final Map<String, Integer> enchantmentIds = new HashMap<>();
    
    @Getter
    private static final Map<Integer, String> networkIdToEnchantmentName = new HashMap<>();

    private static final List<NbtMap> dimensionTypes = new ArrayList<>();
    private static int currentDimensionIndex = 0;

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
        } else if (registryName.equals("minecraft:dimension_type")) {
            dimensionTypes.clear();
            int index = 0;
            for (var entry : registryPacket.getEntries()) {
                NbtMap data = entry.getData();
                // Entries with null data (known packs) must still occupy their slot:
                // the dimension id from login/respawn is an index into the full registry.
                dimensionTypes.add(data);
                if (data != null) {
                    log.debug("Dimension type [{}]: {} → min_y={}, height={}",
                            index, entry.getId(), data.getInt("min_y"), data.getInt("height"));
                }
                index++;
            }
        }
    }

    static void setCurrentDimension(int index) {
        currentDimensionIndex = index;
    }

    public static int getMinWorldY() {
        if (currentDimensionIndex >= 0 && currentDimensionIndex < dimensionTypes.size()) {
            NbtMap data = dimensionTypes.get(currentDimensionIndex);
            if (data != null && data.containsKey("min_y")) return data.getInt("min_y");
        }
        return -64; // fallback for vanilla overworld
    }

    public static int getMaxWorldY() {
        if (currentDimensionIndex >= 0 && currentDimensionIndex < dimensionTypes.size()) {
            NbtMap data = dimensionTypes.get(currentDimensionIndex);
            if (data != null && data.containsKey("min_y") && data.containsKey("height")) {
                return data.getInt("min_y") + data.getInt("height") - 1;
            }
        }
        return 319; // fallback for vanilla overworld
    }
}
