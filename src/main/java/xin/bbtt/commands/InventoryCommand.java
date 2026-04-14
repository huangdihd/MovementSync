package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class InventoryCommand extends Command {
    @Override
    public String getName() {
        return "inventory";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"inventory", "inv"};
    }

    @Override
    public String getDescription() {
        return "Show inventory";
    }

    @Override
    public String getUsage() {
        return "inventory";
    }
}
