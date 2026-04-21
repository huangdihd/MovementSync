package xin.bbtt.commands;

import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponents;
import org.geysermc.mcprotocollib.protocol.data.game.item.component.ItemEnchantments;
import xin.bbtt.MovementSync;
import xin.bbtt.inventory.EnchantmentRegistry;
import xin.bbtt.inventory.ItemRegistry;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.LangManager;

import java.util.Map;

public class InventoryCommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        ItemStack[] items = MovementSync.INSTANCE.getInventoryManager().getInventory();
        if (items == null) {
            MovementSync.getLogger().info(LangManager.get("movementsync.command.inventory.not_loaded"));
            return;
        }

        MovementSync.getLogger().info(LangManager.get("movementsync.command.inventory.header"));
        for (int i = 0; i < items.length; i++) {
            ItemStack item = items[i];
            if (item == null || item.getId() == 0) continue;

            ItemRegistry.ItemEntry entry = ItemRegistry.Instance.getItem(item.getId());
            String name = entry != null ? entry.getDisplayName() : "Unknown (" + item.getId() + ")";
            
            StringBuilder sb = new StringBuilder();
            sb.append(LangManager.get("movementsync.command.inventory.entry", i, name, item.getAmount()));
            
            appendEnchantments(item, sb);
            MovementSync.getLogger().info(sb.toString());
        }
    }

    private void appendEnchantments(ItemStack item, StringBuilder sb) {
        try {
            DataComponents components = item.getDataComponentsPatch();
            if (components == null) return;

            ItemEnchantments enchantmentsObj = components.get(DataComponentTypes.ENCHANTMENTS);
            if (enchantmentsObj == null) return;

            Map<Integer, Integer> enchantments = enchantmentsObj.getEnchantments();
            if (enchantments.isEmpty()) return;

            sb.append(" {");
            boolean first = true;
            for (Map.Entry<Integer, Integer> ench : enchantments.entrySet()) {
                if (!first) sb.append(", ");
                EnchantmentRegistry.EnchantmentEntry eEntry = EnchantmentRegistry.Instance.getByNetworkId(ench.getKey());
                String eName = eEntry != null ? eEntry.getDisplayName() : "ID:" + ench.getKey();
                sb.append(eName).append(" ").append(ench.getValue());
                first = false;
            }
            sb.append("}");
        } catch (Exception ignored) {}
    }
}
