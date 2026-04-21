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
                int containerId = MovementSync.INSTANCE.getInventoryManager().getCurrentContainerId();
                ItemStack[] items = MovementSync.INSTANCE.getInventoryManager().getOpenContainer();
                if (items == null) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.container.no_open"));
                    return;
                }
                MovementSync.getLogger().info(LangManager.get("movementsync.command.container.header", containerId));
                for (int i = 0; i < items.length; i++) {
                    ItemStack item = items[i];
                    if (item != null) {
                        MovementSync.getLogger().info(LangManager.get("movementsync.command.inventory.entry", i, item.getId(), item.getAmount()));
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
