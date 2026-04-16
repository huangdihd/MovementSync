package xin.bbtt.commands;

import org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.InteractAction;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundInteractPacket;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.command.SubCommandExecutor;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.movements.InteractEntityMovement;

public class InteractEntityCommandExecutor extends SubCommandExecutor {
    public InteractEntityCommandExecutor() {
        registerSubCommand("attack", createInteractExecutor(InteractAction.ATTACK));
        registerSubCommand("interact", createInteractExecutor(InteractAction.INTERACT));
        registerSubCommand("interact_at", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                if (args.length < 4) return;
                try {
                    int id = Integer.parseInt(args[0]);
                    double x = Double.parseDouble(args[1]);
                    double y = Double.parseDouble(args[2]);
                    double z = Double.parseDouble(args[3]);
                    MovementSync.Instance.getMovementController().addMovement(new InteractEntityMovement(id, InteractAction.INTERACT_AT, new org.joml.Vector3d(x, y, z)));
                    MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.interactentity.success", InteractAction.INTERACT_AT, id));
                } catch (Exception ignored) {}
            }

            @Override
            public java.util.List<String> onTabComplete(Command command, String label, String[] args) {
                if (args.length == 1) {
                    return MovementSync.Instance.getWorld().getEntities().keySet().stream().map(String::valueOf).filter(s -> s.startsWith(args[0])).collect(java.util.stream.Collectors.toList());
                }
                return java.util.Collections.emptyList();
            }

            @Override
            public org.jline.utils.AttributedStyle[] onHighlight(Command command, String label, String[] args) {
                org.jline.utils.AttributedStyle[] styles = new org.jline.utils.AttributedStyle[args.length];
                if (args.length > 0) styles[0] = new org.jline.utils.AttributedStyle().foreground(org.jline.utils.AttributedStyle.CYAN);
                for(int i=1; i<Math.min(args.length, 4); i++) styles[i] = new org.jline.utils.AttributedStyle().foreground(org.jline.utils.AttributedStyle.YELLOW);
                return styles;
            }
        });
    }

    private CommandExecutor createInteractExecutor(InteractAction action) {
        return new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                if (args.length < 1) return;
                try {
                    int id = Integer.parseInt(args[0]);
                    MovementSync.Instance.getMovementController().addMovement(new InteractEntityMovement(id, action));
                    MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.interactentity.success", action, id));
                } catch (Exception ignored) {}
            }

            @Override
            public java.util.List<String> onTabComplete(Command command, String label, String[] args) {
                if (args.length == 1) {
                    return MovementSync.Instance.getWorld().getEntities().keySet().stream().map(String::valueOf).filter(s -> s.startsWith(args[0])).collect(java.util.stream.Collectors.toList());
                }
                return java.util.Collections.emptyList();
            }

            @Override
            public org.jline.utils.AttributedStyle[] onHighlight(Command command, String label, String[] args) {
                org.jline.utils.AttributedStyle[] styles = new org.jline.utils.AttributedStyle[args.length];
                if (args.length > 0) styles[0] = new org.jline.utils.AttributedStyle().foreground(org.jline.utils.AttributedStyle.CYAN);
                return styles;
            }
        };
    }

    @Override
    protected void onNoSubCommand(Command command, String label) {
        MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
    }
}
