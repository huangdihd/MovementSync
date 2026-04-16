package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class JumpCommand extends Command {
    @Override
    public String getName() {
        return "jump";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"jump"};
    }

    @Override
    public String getDescription() {
        return "Perform a jump";
    }

    @Override
    public String getUsage() {
        return "jump";
    }
}
