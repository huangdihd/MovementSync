package xin.bbtt.commands;

import org.joml.Vector3d;
import org.jline.utils.AttributedStyle;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.TabHighlightExecutor;

import java.util.List;

public class LookAtCommandExecutor extends TabHighlightExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        if (args.length != 3) return;
        try {
            double x = Double.parseDouble(args[0]);
            double y = Double.parseDouble(args[1]);
            double z = Double.parseDouble(args[2]);
            Vector3d target = new Vector3d(x, y, z);
            MovementSync.Instance.lookAt(target);
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
