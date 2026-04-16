package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class EntitiesCommand extends Command {
    @Override
    public String getName() {
        return "entities";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"entities"};
    }

    @Override
    public String getDescription() {
        return "List entities in the world";
    }

    @Override
    public String getUsage() {
        return "entities [filter]";
    }
}
