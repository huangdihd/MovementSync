package xin.bbtt.commands;

import xin.bbtt.Entity.Entity;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.LangManager;

public class FollowCommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        if (args.length < 1) {
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
            return;
        }

        if (args[0].equalsIgnoreCase("stop")) {
            MovementSync.Instance.setFollowTargetId(-1);
            MovementSync.Instance.getMovementController().finishCurrentMovement();
            MovementSync.Instance.getLogger().info("Stopped following.");
            return;
        }

        try {
            int id = Integer.parseInt(args[0]);
            Entity target = MovementSync.Instance.getWorld().getEntity(id);
            if (target == null) {
                MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.lookentity.not_found", id));
                return;
            }

            MovementSync.Instance.setFollowTargetId(id);
            MovementSync.Instance.triggerAutoRepath();
            MovementSync.Instance.getLogger().info("Now following entity " + id);
        } catch (NumberFormatException e) {
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
        }
    }
}
