package xin.bbtt.commands;

import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;

import xin.bbtt.mcbot.LangManager;

public class WhereAmICommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        var p = MovementSync.Instance.position.get();
        MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.whereami.response", p.x, p.y, p.z, MovementSync.Instance.yaw.get(), MovementSync.Instance.pitch.get()));
    }
}
