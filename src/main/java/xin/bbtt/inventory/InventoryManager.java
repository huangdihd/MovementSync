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
    @Getter @Setter
    private int heldSlot = 0;
    
    private final Map<Integer, ItemStack[]> containers = new ConcurrentHashMap<>();

    public ItemStack getHeldItem() {
        ItemStack[] inventory = getInventory();
        if (inventory != null && inventory.length >= 45 && heldSlot >= 0 && heldSlot < 9) {
            return inventory[36 + heldSlot];
        }
        return null;
    }

    public int findBlock() {
        ItemStack[] inventory = getInventory();
        if (inventory == null || inventory.length < 45) return -1;
        // Search hotbar (36-44). An item id and a block-state id belong to
        // separate registries, so resolve the item to its name and look the
        // block up by name. Feeding the item id straight into isSolidBlock()
        // matches unrelated blocks and reports an empty hotbar (air, id 0) as
        // holding placeable blocks — which makes the planner build bridge/
        // pillar paths that can't actually be executed.
        for (int i = 36; i < 45; i++) {
            ItemStack stack = inventory[i];
            if (stack == null) continue;
            ItemRegistry.ItemEntry item = ItemRegistry.Instance.getItem(stack.getId());
            if (item == null) continue;
            if (xin.bbtt.Block.BlockStateParser.Instance.isSolidBlockByName(item.getName())) {
                return i - 36;
            }
        }
        return -1;
    }

    public int findBestTool(String material) {
        ItemStack[] inventory = getInventory();
        if (inventory == null) return -1;
        
        int bestSlot = -1;
        double maxMultiplier = 1.0;

        for (int i = 36; i < 45; i++) {
            if (inventory[i] == null) continue;
            double multiplier = ToolUtils.getToolMultiplier(inventory[i].getId(), material);
            if (multiplier > maxMultiplier) {
                maxMultiplier = multiplier;
                bestSlot = i - 36;
            }
        }
        return bestSlot;
    }

    public void switchToSlot(int slot) {
        if (slot < 0 || slot > 8) return;
        if (heldSlot == slot) return;
        xin.bbtt.mcbot.Bot.INSTANCE.getSession().send(new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSetCarriedItemPacket(slot));
        heldSlot = slot;
    }

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
