package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundPlayerPositionPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundAcceptTeleportationPacket;
import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.events.TeleportEvent;
import xin.bbtt.mcbot.Bot;

import static xin.bbtt.tasks.updateMotionTask.checkOnGround;

public class TeleportPacketListener extends SessionAdapter {
    @Override
    public synchronized void packetReceived(Session session, Packet packet) {
        if (!(packet instanceof ClientboundPlayerPositionPacket playerPositionPacket)) return;
        Vector3d position = new Vector3d(playerPositionPacket.getPosition().getX(), playerPositionPacket.getPosition().getY(), playerPositionPacket.getPosition().getZ());
        TeleportEvent teleportEvent = new TeleportEvent(playerPositionPacket.getId(), position);
        Bot.Instance.getPluginManager().events().callEvent(teleportEvent);
        if (teleportEvent.isDefaultActionCancelled()) return;
        MovementSync.Instance.position.set(position);
        MovementSync.Instance.pitch.set(playerPositionPacket.getXRot());
        MovementSync.Instance.yaw.set(playerPositionPacket.getYRot());
        session.send(new ServerboundAcceptTeleportationPacket(playerPositionPacket.getId()));
        MovementSync.Instance.movementController.cancelAll();
        MovementSync.Instance.velocity.set(new Vector3d());
        checkOnGround();
    }
}
