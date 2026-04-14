package xin.bbtt.commands;

import org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.InteractAction;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundInteractPacket;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.command.Command;
import xin.bbtt.mcbot.command.CommandExecutor;
import xin.bbtt.mcbot.command.SubCommandExecutor;
import xin.bbtt.mcbot.LangManager;

public class InteractEntityCommandExecutor extends SubCommandExecutor {
    public InteractEntityCommandExecutor() {
        registerSubCommand("attack", createInteractExecutor(InteractAction.ATTACK));
        registerSubCommand("interact", createInteractExecutor(InteractAction.INTERACT));
    }

    private CommandExecutor createInteractExecutor(InteractAction action) {
        return new CommandExecutor() {
            @Override
            public void onCommand(Command command, String label, String[] args) {
                if (args.length < 1) return;
                int id = Integer.parseInt(args[0]);
                Bot.Instance.getSession().send(new ServerboundInteractPacket(id, action, Hand.MAIN_HAND, false));
                MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.interactentity.success", action, id));
            }
        };
    }

    @Override
    protected void onNoSubCommand(Command command, String label) {
        MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.common.usage", command.getUsage()));
    }
}
