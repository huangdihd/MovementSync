package xin.bbtt.commands;

import org.joml.Vector3d;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;

public class GetBlockAtCommandExecutor extends CommandExecutor {
    private static final Logger log = LoggerFactory.getLogger(GetBlockAtCommandExecutor.class.getSimpleName());
    @Override
    public void onCommand(Command command, String s, String[] args) {
        if (args.length != 3) {
            return;
        }
        int z;
        int x;
        int y;
        try {
            x = Integer.parseInt(args[0]);
            y = Integer.parseInt(args[1]);
            z = Integer.parseInt(args[2]);
        } catch (NumberFormatException e) {
            return;
        }
        log.info(LangManager.get("movementsync.command.getblockat.response", BlockStateParser.Instance.parseStateId(MovementSync.Instance.getWorld().getBlockAt(new Vector3d(x, y, z)))));
    }
}
