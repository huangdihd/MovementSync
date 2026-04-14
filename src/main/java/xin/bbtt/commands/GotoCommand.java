package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class GotoCommand extends Command {
    @Override
    public String getName() {
        return "goto";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"goto", "move"};
    }

    @Override
    public String getDescription() {
        return "Pathfind to a target position";
    }

    @Override
    public String getUsage() {
        return "goto <x> <y> <z>";
    }
}
