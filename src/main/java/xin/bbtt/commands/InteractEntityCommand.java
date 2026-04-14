package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class InteractEntityCommand extends Command {
    @Override
    public String getName() {
        return "interactentity";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"interactentity"};
    }

    @Override
    public String getDescription() {
        return "Interact with an entity";
    }

    @Override
    public String getUsage() {
        return "interactentity <id> <attack|interact>";
    }
}
