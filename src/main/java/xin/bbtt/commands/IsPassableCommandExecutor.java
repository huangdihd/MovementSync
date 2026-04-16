package xin.bbtt.commands;

import org.joml.Vector3d;
import org.jline.utils.AttributedStyle;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.TabHighlightExecutor;

import java.util.List;

public class IsPassableCommandExecutor extends TabHighlightExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        if (args.length < 3) return;
        try {
            int x = Integer.parseInt(args[0]);
            int y = Integer.parseInt(args[1]);
            int z = Integer.parseInt(args[2]);
            boolean passable = MovementSync.Instance.getWorld().isPassable(new Vector3d(x, y, z));
            MovementSync.Instance.getLogger().info("Passable at " + x + ", " + y + ", " + z + ": " + passable);
        } catch (NumberFormatException ignored) {}
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
