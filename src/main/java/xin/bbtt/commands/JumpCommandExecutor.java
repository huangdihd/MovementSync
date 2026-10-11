package xin.bbtt.commands;

import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;

public class JumpCommandExecutor extends CommandExecutor {
    @Override
    public void onCommand(Command command, String label, String[] args) {
        // Jump is an input pulse, not an action queued behind a timed walk.
        MovementSync.INSTANCE.jump();
    }
}
