package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class GotoCommand extends Command {
    @Override
    public String getName() {
        return "goto";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"goto"};
    }

    @Override
    public String getDescription() {
        return "Pathfind to a coordinate";
    }

    @Override
    public String getUsage() {
        return "goto <x> <y> <z>";
    }
}
