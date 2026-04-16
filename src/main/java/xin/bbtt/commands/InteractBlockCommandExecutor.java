package xin.bbtt.commands;

import org.cloudburstmc.math.vector.Vector3i;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.command.SubCommandExecutor;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.movements.DigBlockMovement;

public class InteractBlockCommandExecutor extends SubCommandExecutor {
    public InteractBlockCommandExecutor() {
        registerSubCommand("dig", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                if (args.length < 3) return;
                try {
                    int x = Integer.parseInt(args[0]);
                    int y = Integer.parseInt(args[1]);
                    int z = Integer.parseInt(args[2]);
                    Vector3i pos = Vector3i.from(x, y, z);
                    
                    xin.bbtt.Block.BlockState state = MovementSync.Instance.getWorld().getBlockStateAt(new org.joml.Vector3d(x, y, z));
                    org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack held = MovementSync.Instance.getInventoryManager().getHeldItem();
                    double breakTicks = xin.bbtt.inventory.ToolUtils.calculateBreakTicks(held, state);
                    
                    if (breakTicks < 0) {
                        MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.interactblock.dig.not_diggable", x, y, z));
                        return;
                    }
                    
                    long estimatedTimeMs = (long)(Math.ceil(breakTicks) * 50);
                    MovementSync.Instance.getMovementController().addMovement(new DigBlockMovement(pos));
                    MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.interactblock.dig.started", 
                        state.blockName(), x, y, z, estimatedTimeMs / 1000.0));
                } catch (NumberFormatException e) {
                    MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
                }
            }

            @Override
            public org.jline.utils.AttributedStyle[] onHighlight(Command command, String label, String[] args) {
                org.jline.utils.AttributedStyle[] styles = new org.jline.utils.AttributedStyle[args.length];
                for(int i=0; i<Math.min(args.length, 3); i++) styles[i] = new org.jline.utils.AttributedStyle().foreground(org.jline.utils.AttributedStyle.YELLOW);
                return styles;
            }
        });
    }

    @Override
    protected void onNoSubCommand(Command command, String label) {
        MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
    }
}
