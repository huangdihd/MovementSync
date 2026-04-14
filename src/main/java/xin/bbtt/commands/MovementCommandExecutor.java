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
                MovementSync.Instance.getMovementController().pause();
                MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.movement.paused"));
            }
        });

        registerSubCommand("resume", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                MovementSync.Instance.getMovementController().resume();
                MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.movement.resumed"));
            }
        });

        registerSubCommand("cancel", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                MovementSync.Instance.getMovementController().cancelAll();
                MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.movement.cancelled"));
            }
        });

        registerSubCommand("cancel_current", new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                MovementSync.Instance.getMovementController().finishCurrentMovement();
                MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.movement.cancel_current"));
            }
        });
    }

    @Override
    protected void onNoSubCommand(Command command, String label) {
        MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
    }
}
