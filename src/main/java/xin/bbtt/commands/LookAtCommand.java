package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class LookAtCommand extends Command {
    @Override
    public String getName() {
        return "lookat";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"lookat"};
    }

    @Override
    public String getDescription() {
        return "Look at a coordinate";
    }

    @Override
    public String getUsage() {
        return "lookat <x> <y> <z>";
    }
}
