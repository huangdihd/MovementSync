package xin.bbtt.commands;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.Entity.Entity;

public class LookAtEntityCommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        if (args.length < 1) {
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
            return;
        }

        try {
            int entityId = Integer.parseInt(args[0]);
            Entity entity = MovementSync.Instance.getWorld().getEntity(entityId);

            if (entity == null) {
                MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.lookentity.not_found", entityId));
                return;
            }

            // Aim for the center/head of the entity. Since we don't have accurate dimensions, 
            // we assume a default height offset.
            Vector3d targetPos = new Vector3d(entity.getPosition()).add(0, 1.5, 0);
            MovementSync.Instance.lookAt(targetPos);
            
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.lookentity.success", entityId, entity.getType()));
        } catch (NumberFormatException e) {
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
        }
    }
}
