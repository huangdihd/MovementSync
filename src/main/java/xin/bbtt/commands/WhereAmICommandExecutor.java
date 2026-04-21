package xin.bbtt.commands;

import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;

import xin.bbtt.mcbot.LangManager;

public class WhereAmICommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        var p = MovementSync.INSTANCE.position.get();
        MovementSync.getLogger().info(LangManager.get("movementsync.command.whereami.response", p.x, p.y, p.z, MovementSync.INSTANCE.yaw.get(), MovementSync.INSTANCE.pitch.get()));
    }
}
