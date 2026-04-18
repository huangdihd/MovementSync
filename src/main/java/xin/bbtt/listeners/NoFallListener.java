package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.event.session.PacketSendingEvent;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
import xin.bbtt.Config;
import xin.bbtt.MovementSync;

public class NoFallListener extends SessionAdapter {
    @Override
    public void packetSending(PacketSendingEvent event) {
        if (!(event.getPacket() instanceof ServerboundMovePlayerPosRotPacket movePlayerPosRotPacket)) return;
        if (!Config.noFall) return;
        if (MovementSync.Instance.velocity.get().y > -0.5) return;
        event.setPacket(movePlayerPosRotPacket.withOnGround(true));
    }

}
