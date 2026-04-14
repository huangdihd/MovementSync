package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class InteractBlockCommand extends Command {
    @Override
    public String getName() {
        return "interactblock";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"interactblock"};
    }

    @Override
    public String getDescription() {
        return "Interact with a block";
    }

    @Override
    public String getUsage() {
        return "interactblock <x> <y> <z> <start_dig|finish_dig|cancel_dig>";
    }
}
