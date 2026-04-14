package xin.bbtt.commands;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;

public class WhereAmICommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String s, String[] strings) {
        Vector3d position = new Vector3d(MovementSync.Instance.position.get());
        MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.whereami.response"), Bot.Instance.getServer(), position.x, position.y, position.z);
    }
}
