package xin.bbtt.commands;

import org.joml.Vector3d;
import org.jline.utils.AttributedStyle;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.TabHighlightExecutor;
import xin.bbtt.mcbot.LangManager;

import java.util.List;

public class LookAtCommandExecutor extends TabHighlightExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        if (args.length != 3) return;
        try {
            double x = Double.parseDouble(args[0]);
            double y = Double.parseDouble(args[1]);
            double z = Double.parseDouble(args[2]);
            if (!xin.bbtt.world.World.isWithinWorldBounds((int) y)) {
                MovementSync.getLogger().info(LangManager.get("movementsync.command.common.out_of_bounds", (int) y, xin.bbtt.world.World.getMinWorldY(), xin.bbtt.world.World.getMaxWorldY()));
                return;
            }
            Vector3d target = new Vector3d(x, y, z);
            MovementSync.INSTANCE.lookAt(target);
        } catch (NumberFormatException e) {
            MovementSync.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
        }
    }

    @Override
    public List<String> onTabComplete(Command command, String label, String[] args) {
        return List.of();
    }

    @Override
    public AttributedStyle[] onHighlight(Command command, String label, String[] args) {
        AttributedStyle[] styles = new AttributedStyle[args.length];
        for (int i = 0; i < Math.min(args.length, 3); i++) {
            styles[i] = new AttributedStyle().foreground(AttributedStyle.YELLOW);
        }
        return styles;
    }
}
