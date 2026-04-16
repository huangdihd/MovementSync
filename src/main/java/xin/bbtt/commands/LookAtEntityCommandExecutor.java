package xin.bbtt.commands;

import org.joml.Vector3d;
import org.jline.utils.AttributedStyle;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.TabHighlightExecutor;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.Entity.Entity;

import java.util.List;
import java.util.stream.Collectors;

public class LookAtEntityCommandExecutor extends TabHighlightExecutor {
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

            Vector3d targetPos = new Vector3d(entity.getPosition()).add(0, 1.5, 0);
            MovementSync.Instance.lookAt(targetPos);
            
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.lookentity.success", entityId, entity.getType()));
        } catch (NumberFormatException e) {
            MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
        }
    }

    @Override
    public List<String> onTabComplete(Command command, String label, String[] args) {
        if (args.length == 1) {
            return MovementSync.Instance.getWorld().getEntities().keySet().stream()
                    .map(String::valueOf)
                    .filter(s -> s.startsWith(args[0]))
                    .collect(Collectors.toList());
        }
        return List.of();
    }

    @Override
    public AttributedStyle[] onHighlight(Command command, String label, String[] args) {
        AttributedStyle[] styles = new AttributedStyle[args.length];
        if (args.length > 0) {
            styles[0] = new AttributedStyle().foreground(AttributedStyle.CYAN);
        }
        return styles;
    }
}
