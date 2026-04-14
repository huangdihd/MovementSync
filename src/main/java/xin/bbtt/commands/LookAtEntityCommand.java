package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class LookAtEntityCommand extends Command {
    @Override
    public String getName() {
        return "lookentity";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"lookentity", "le"};
    }

    @Override
    public String getDescription() {
        return "Look at a specific entity by ID";
    }

    @Override
    public String getUsage() {
        return "lookentity <id>";
    }
}
