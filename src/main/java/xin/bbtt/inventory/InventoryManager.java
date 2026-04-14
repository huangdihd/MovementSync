package xin.bbtt.inventory;

import lombok.Getter;
import lombok.Setter;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class InventoryManager {
    @Getter @Setter
    private int currentContainerId = 0;
    @Getter @Setter
    private int currentStateId = 0;
    
    private final Map<Integer, ItemStack[]> containers = new ConcurrentHashMap<>();

    public void setContainerItems(int containerId, ItemStack[] items) {
        containers.put(containerId, items);
    }

    public void setSlot(int containerId, int slot, ItemStack item) {
        ItemStack[] items = containers.get(containerId);
        if (items != null && slot >= 0 && slot < items.length) {
            items[slot] = item;
        }
    }

    public ItemStack[] getInventory() {
        return containers.get(0);
    }

    public ItemStack[] getOpenContainer() {
        return containers.get(currentContainerId);
    }

    public ItemStack[] getContainer(int id) {
        return containers.get(id);
    }
}
