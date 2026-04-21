package xin.bbtt.commands;

import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.command.SubCommandExecutor;
import xin.bbtt.mcbot.LangManager;

public class MovementCommandExecutor extends SubCommandExecutor {
    public MovementCommandExecutor() {
        registerSubCommand("pause", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                MovementSync.INSTANCE.getMovementController().pause();
                MovementSync.getLogger().info(LangManager.get("movementsync.command.movement.paused"));
            }
        });

        registerSubCommand("resume", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                MovementSync.INSTANCE.getMovementController().resume();
                MovementSync.getLogger().info(LangManager.get("movementsync.command.movement.resumed"));
            }
        });

        registerSubCommand("cancel", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                MovementSync.INSTANCE.getMovementController().cancelAll();
                MovementSync.getLogger().info(LangManager.get("movementsync.command.movement.cancelled"));
            }
        });

        registerSubCommand("cancel_current", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                MovementSync.INSTANCE.getMovementController().finishCurrentMovement();
                MovementSync.getLogger().info(LangManager.get("movementsync.command.movement.cancel_current"));
            }
        });

        registerSubCommand("list", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                java.util.List<xin.bbtt.movement.Movement> list = MovementSync.INSTANCE.getMovementController().getMovements();
                xin.bbtt.movement.Movement current = MovementSync.INSTANCE.getMovementController().getCurrentMovement();
                MovementSync.getLogger().info(LangManager.get("movementsync.command.movement.list.current", current != null ? current.getClass().getSimpleName() : "None"));
                MovementSync.getLogger().info(LangManager.get("movementsync.command.movement.list.queue", list.size()));
                for (int i = 0; i < list.size(); i++) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.movement.list.entry", i, list.get(i).getClass().getSimpleName()));
                }
            }
        });

        registerSubCommand("remove", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                if (args.length < 1) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.common.usage", "movement remove <index>"));
                    return;
                }
                try {
                    int index = Integer.parseInt(args[0]);
                    boolean removed = MovementSync.INSTANCE.getMovementController().removeMovement(index);
                    if (removed) {
                        MovementSync.getLogger().info(LangManager.get("movementsync.command.movement.remove.success", index));
                    } else {
                        MovementSync.getLogger().info(LangManager.get("movementsync.command.movement.remove.invalid"));
                    }
                } catch (NumberFormatException e) {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.movement.remove.format"));
                }
            }

            @Override
            public java.util.List<String> onTabComplete(Command command, String label, String[] args) {
                if (args.length == 1) {
                    int size = MovementSync.INSTANCE.getMovementController().getMovements().size();
                    java.util.List<String> list = new java.util.ArrayList<>();
                    for (int i = 0; i < size; i++) {
                        list.add(String.valueOf(i));
                    }
                    return list;
                }
                return java.util.Collections.emptyList();
            }

            @Override
            public org.jline.utils.AttributedStyle[] onHighlight(Command command, String label, String[] args) {
                org.jline.utils.AttributedStyle[] styles = new org.jline.utils.AttributedStyle[args.length];
                if (args.length > 0) {
                    styles[0] = new org.jline.utils.AttributedStyle().foreground(org.jline.utils.AttributedStyle.YELLOW);
                }
                return styles;
            }
        });
    }

    @Override
    protected void onNoSubCommand(Command command, String label) {
        MovementSync.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
    }
}
