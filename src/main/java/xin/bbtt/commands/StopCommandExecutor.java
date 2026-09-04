package xin.bbtt.commands;

import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.LangManager;

public class StopCommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        MovementSync.INSTANCE.cancelNavigation();
        MovementSync.getLogger().info(LangManager.get("movementsync.command.stop.response"));
    }
}
