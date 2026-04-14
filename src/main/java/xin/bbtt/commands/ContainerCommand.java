package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class ContainerCommand extends Command {
    @Override
    public String getName() {
        return "container";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"container", "ct"};
    }

    @Override
    public String getDescription() {
        return "Show current container content";
    }

    @Override
    public String getUsage() {
        return "container [list]";
    }
}
