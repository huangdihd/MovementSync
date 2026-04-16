package xin.bbtt.inventory;

import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponents;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.ItemEnchantments;
import xin.bbtt.Block.BlockState;
import xin.bbtt.listeners.RegistryDataListener;

import java.util.Map;

public class ToolUtils {
    public static double getToolMultiplier(int itemId, String material) {
        if (itemId == 0) return 1.0;
        
        ItemRegistry.ItemEntry item = ItemRegistry.Instance.getItem(itemId);
        if (item == null) return 1.0;

        String itemName = item.getName();
        if (!isCorrectTool(itemName, material)) return 1.0;

        if (itemName.contains("gold_")) return 12.0;
        if (itemName.contains("netherite_")) return 9.0;
        if (itemName.contains("diamond_")) return 8.0;
        if (itemName.contains("iron_")) return 6.0;
        if (itemName.contains("stone_")) return 4.0;
        if (itemName.contains("wooden_")) return 2.0;
        
        return 1.0;
    }

    private static boolean isCorrectTool(String itemName, String material) {
        if (material == null) return true;
        if (material.contains("pickaxe") && itemName.contains("_pickaxe")) return true;
        if (material.contains("shovel") && itemName.contains("_shovel")) return true;
        if (material.contains("axe") && itemName.contains("_axe")) return true;
        if (material.contains("hoe") && itemName.contains("_hoe")) return true;
        return material.contains("shears") && itemName.contains("shears");
    }

    public static double calculateBreakTicks(ItemStack held, BlockState state) {
        if (!state.diggable()) return -1;
        
        int toolId = (held != null) ? held.getId() : 0;
        double toolMultiplier = getToolMultiplier(toolId, state.material());
        
        int efficiencyLevel = getEfficiencyLevel(held);
        if (toolMultiplier > 1.0 && efficiencyLevel > 0) {
            toolMultiplier += (efficiencyLevel * efficiencyLevel + 1);
        }
        
        double speed = toolMultiplier; // Haste omitted for now
        double hardness = state.hardness();
        
        if (toolMultiplier > 1.0) {
            return (hardness * 1.5 * 20) / speed;
        }
        return (hardness * 5.0 * 20);
    }

    public static int getEfficiencyLevel(ItemStack item) {
        if (item == null) return 0;
        try {
            DataComponents components = item.getDataComponentsPatch();
            if (components == null) return 0;

            ItemEnchantments enchantmentsObj = components.get(DataComponentTypes.ENCHANTMENTS);
            if (enchantmentsObj == null) return 0;

            Map<Integer, Integer> enchantments = enchantmentsObj.getEnchantments();
            Integer effId = RegistryDataListener.getEnchantmentIds().get("minecraft:efficiency");
            
            if (effId != null && enchantments.containsKey(effId)) {
                return enchantments.get(effId);
            }
        } catch (Exception ignored) {}
        return 0;
    }
}
