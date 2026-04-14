package xin.bbtt.commands;

import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.command.SubCommandExecutor;
import xin.bbtt.mcbot.LangManager;

public class ContainerCommandExecutor extends SubCommandExecutor {
    public ContainerCommandExecutor() {
        registerSubCommand("list", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                int containerId = MovementSync.Instance.getInventoryManager().getCurrentContainerId();
                ItemStack[] items = MovementSync.Instance.getInventoryManager().getOpenContainer();
                if (items == null || containerId == 0) {
                    MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.container.no_open"));
                    return;
                }
                MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.container.header", containerId));
                for (int i = 0; i < items.length; i++) {
                    ItemStack item = items[i];
                    if (item != null && item.getId() != 0) {
                        MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.inventory.entry", i, item.getId(), item.getAmount()));
                    }
                }
            }
        });
    }

    @Override
    protected void onNoSubCommand(Command command, String label) {
        onCommand(command, label, new String[]{"list"});
    }
}
