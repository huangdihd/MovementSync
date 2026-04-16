package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class LookAtEntityCommand extends Command {
    @Override
    public String getName() {
        return "lookatentity";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"lookatentity"};
    }

    @Override
    public String getDescription() {
        return "Look at an entity by ID";
    }

    @Override
    public String getUsage() {
        return "lookatentity <entityId>";
    }
}
