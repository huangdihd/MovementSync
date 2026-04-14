package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class MovementCommand extends Command {
    @Override
    public String getName() {
        return "movement";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"movement"};
    }

    @Override
    public String getDescription() {
        return "Control the MovementController (pause/resume/cancel)";
    }

    @Override
    public String getUsage() {
        return "movement <pause|resume|cancel>";
    }
}
