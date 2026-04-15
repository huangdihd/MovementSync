package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class FollowCommand extends Command {
    @Override
    public String getName() { return "follow"; }
    @Override
    public String[] getAliases() { return new String[]{"follow"}; }
    @Override
    public String getDescription() { return "Follow a specific entity"; }
    @Override
    public String getUsage() { return "follow <id|stop>"; }
}
