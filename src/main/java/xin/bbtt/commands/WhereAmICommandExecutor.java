package xin.bbtt.commands;

import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;

public class WhereAmICommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        var p = MovementSync.Instance.position.get();
        MovementSync.Instance.getLogger().info(String.format("Position: %.2f, %.2f, %.2f | Yaw: %.1f | Pitch: %.1f", p.x, p.y, p.z, MovementSync.Instance.yaw.get(), MovementSync.Instance.pitch.get()));
    }
}
