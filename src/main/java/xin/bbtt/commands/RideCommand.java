package xin.bbtt.commands;

import xin.bbtt.mcbot.command.Command;

public class RideCommand extends Command {
    @Override
    public String getName() {
        return "ride";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"ride", "mount"};
    }

    @Override
    public String getDescription() {
        return "Ride an entity or dismount";
    }

    @Override
    public String getUsage() {
        return "ride <entityId> | ride dismount";
    }
}
