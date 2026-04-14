package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class IsPassableCommand extends Command {
    @Override
    public String getName() {
        return "ispassable";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"ispassable"};
    }

    @Override
    public String getDescription() {
        return "Check if a block is passable";
    }

    @Override
    public String getUsage() {
        return "ispassable <x> <y> <z>";
    }
}
