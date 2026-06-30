package xin.bbtt.commands;

import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.TabHighlightExecutor;
import xin.bbtt.mcbot.LangManager;

import java.util.List;
import java.util.stream.Collectors;

public class FollowCommandExecutor extends TabHighlightExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        if (args.length < 1) {
            MovementSync.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
            return;
        }

        try {
            int entityId = Integer.parseInt(args[0]);
            double keepDistance = -1;
            if (args.length >= 2) {
                keepDistance = Double.parseDouble(args[1]);
                if (keepDistance <= 0) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
                    return;
                }
            }
            MovementSync.INSTANCE.setFollowTargetId(entityId);
            MovementSync.INSTANCE.triggerAutoRepath(keepDistance);
            MovementSync.getLogger().info(LangManager.get("movementsync.command.follow.success", entityId));
        } catch (NumberFormatException e) {
            MovementSync.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
        }
    }

    @Override
    public List<String> onTabComplete(Command command, String label, String[] args) {
        if (args.length == 1) {
            return MovementSync.INSTANCE.getWorld().getEntities().keySet().stream()
                    .map(String::valueOf)
                    .filter(s -> s.startsWith(args[0]))
                    .collect(Collectors.toList());
        }
        return List.of();
    }

    @Override
    public org.jline.utils.AttributedStyle[] onHighlight(Command command, String label, String[] args) {
        org.jline.utils.AttributedStyle[] styles = new org.jline.utils.AttributedStyle[args.length];
        if (args.length > 0) {
            styles[0] = new org.jline.utils.AttributedStyle().foreground(org.jline.utils.AttributedStyle.CYAN);
        }
        return styles;
    }
}
