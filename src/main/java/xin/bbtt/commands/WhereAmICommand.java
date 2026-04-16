package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class WhereAmICommand extends Command {
    @Override
    public String getName() {
        return "whereami";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"whereami", "pos"};
    }

    @Override
    public String getDescription() {
        return "Show current position and rotation";
    }

    @Override
    public String getUsage() {
        return "whereami";
    }
}
