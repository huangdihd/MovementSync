package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class GetBlockAtCommand extends Command {
    @Override
    public String getName() {
        return "getblockat";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"getblockat", "block"};
    }

    @Override
    public String getDescription() {
        return "Get block info at a coordinate";
    }

    @Override
    public String getUsage() {
        return "getblockat <x> <y> <z>";
    }
}
