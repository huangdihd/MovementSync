package xin.bbtt.commands;

import org.jline.utils.AttributedStyle;
import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.TabHighlightExecutor;

import java.util.List;

public class CanSeeCommandExecutor extends TabHighlightExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        if (args.length != 3) {
            MovementSync.getLogger().info("Usage: cansee <x> <y> <z> (supports floating point)");
            return;
        }
        try {
            double x = Double.parseDouble(args[0]);
            double y = Double.parseDouble(args[1]);
            double z = Double.parseDouble(args[2]);
            
            Vector3d headPos = MovementSync.INSTANCE.getHeadPosition();
            Vector3d target = new Vector3d(x, y, z);
            
            MovementSync.getLogger().info("Checking visibility from {} to {}...", headPos, target);
            
            boolean canSee = MovementSync.INSTANCE.getWorld().canSee(headPos, target);
            if (canSee) {
                MovementSync.getLogger().info("The bot can see the point at ({}, {}, {})", x, y, z);
            } else {
                MovementSync.getLogger().info("The bot CANNOT see the point at ({}, {}, {})", x, y, z);
            }
        } catch (NumberFormatException e) {
            MovementSync.getLogger().info("Invalid coordinates.");
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