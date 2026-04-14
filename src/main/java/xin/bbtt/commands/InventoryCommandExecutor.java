package xin.bbtt.commands;

import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.LangManager;

public class InventoryCommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        ItemStack[] items = MovementSync.Instance.getInventoryManager().getInventory();
        if (items == null) {
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.inventory.not_loaded"));
            return;
        }
        MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.inventory.header"));
        for (int i = 0; i < items.length; i++) {
            ItemStack item = items[i];
            if (item != null && item.getId() != 0) {
                MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.inventory.entry", i, item.getId(), item.getAmount()));
            }
        }
    }
}
