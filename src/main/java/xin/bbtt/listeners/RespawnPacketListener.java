package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.data.game.ClientCommand;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundPlayerCombatKillPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.ServerboundClientCommandPacket;
import xin.bbtt.MovementSync;
import xin.bbtt.events.DeathEvent;
import xin.bbtt.mcbot.Bot;

public class RespawnPacketListener extends SessionAdapter {
    @Override
    public void packetReceived(Session session, Packet packet) {
        if (!(packet instanceof ClientboundPlayerCombatKillPacket playerCombatKillPacket)) return;
        MovementSync.getLogger().info(playerCombatKillPacket.toString());
        MovementSync.INSTANCE.cancelNavigation();
        DeathEvent deathEvent = new DeathEvent(playerCombatKillPacket.getPlayerId(), playerCombatKillPacket.getMessage());
        Bot.INSTANCE.getPluginManager().events().callEvent(deathEvent);
        if (deathEvent.isDefaultActionCancelled()) return;
        session.send(new ServerboundClientCommandPacket(
                ClientCommand.RESPAWN
        ));
    }
}
