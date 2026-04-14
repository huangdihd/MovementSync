package xin.bbtt.commands;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.Block.BlockState;

public class IsPassableCommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        if (args.length < 3) return;
        try {
            double x = Double.parseDouble(args[0]);
            double y = Double.parseDouble(args[1]);
            double z = Double.parseDouble(args[2]);
            Vector3d pos = new Vector3d(x, y, z);
            
            BlockState state = MovementSync.Instance.getWorld().getBlockStateAt(pos);
            boolean passable = state.isPassable();
            
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.ispassable.response", 
                state.blockName(), passable));
        } catch (Exception e) {
            MovementSync.Instance.getLogger().error("Error checking passability", e);
        }
    }
}
