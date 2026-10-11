package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundPlayerPositionPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundAcceptTeleportationPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
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
        Bot.INSTANCE.getPluginManager().events().callEvent(teleportEvent);
        if (teleportEvent.isDefaultActionCancelled()) return;
        MovementSync.INSTANCE.position.set(position);
        MovementSync.INSTANCE.pitch.set(playerPositionPacket.getXRot());
        MovementSync.INSTANCE.yaw.set(playerPositionPacket.getYRot());
        session.send(new ServerboundAcceptTeleportationPacket(playerPositionPacket.getId()));
        // The vanilla client follows the confirmation with a full position packet
        // echoing the teleport target; servers treat the teleport as pending until then.
        session.send(new ServerboundMovePlayerPosRotPacket(
                false, false, position.x, position.y, position.z,
                playerPositionPacket.getYRot(), playerPositionPacket.getXRot()));
        MovementSync.INSTANCE.cancelNavigation();
        // A teleport does not end a glide: the server keeps its gliding flag set and
        // sends no metadata update, so clearing it here would leave the two sides out of sync.
        MovementSync.INSTANCE.velocity.set(new Vector3d());
        checkOnGround();
    }
}
