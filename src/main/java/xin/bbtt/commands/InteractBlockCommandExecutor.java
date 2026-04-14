package xin.bbtt.commands;

import org.cloudburstmc.math.vector.Vector3i;
import org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerAction;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerActionPacket;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.command.SubCommandExecutor;
import xin.bbtt.mcbot.LangManager;

public class InteractBlockCommandExecutor extends SubCommandExecutor {
    public InteractBlockCommandExecutor() {
        registerSubCommand("start_dig", createActionExecutor(PlayerAction.START_DIGGING));
        registerSubCommand("finish_dig", createActionExecutor(PlayerAction.FINISH_DIGGING));
        registerSubCommand("cancel_dig", createActionExecutor(PlayerAction.CANCEL_DIGGING));
    }

    private CommandExecutor createActionExecutor(PlayerAction action) {
        return new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                if (args.length < 3) return;
                int x = Integer.parseInt(args[0]);
                int y = Integer.parseInt(args[1]);
                int z = Integer.parseInt(args[2]);
                Vector3i pos = Vector3i.from(x, y, z);
                Bot.Instance.getSession().send(new ServerboundPlayerActionPacket(action, pos, Direction.UP, 0));
                MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.interactblock.success", action, x, y, z));
            }
        };
    }

    @Override
    protected void onNoSubCommand(Command command, String label) {
        MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
    }
}
