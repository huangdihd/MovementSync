package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundPlayerPositionPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundAcceptTeleportationPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PositionElement;
import org.joml.Vector3d;

import java.util.List;
import xin.bbtt.MovementSync;
import xin.bbtt.events.TeleportEvent;
import xin.bbtt.mcbot.Bot;

import static xin.bbtt.tasks.updateMotionTask.checkOnGround;

public class TeleportPacketListener extends SessionAdapter {
    @Override
    public synchronized void packetReceived(Session session, Packet packet) {
        if (!(packet instanceof ClientboundPlayerPositionPacket playerPositionPacket)) return;
        // Resolve the packet like the vanilla client: every component may be relative to
        // the current state, and the velocity is taken from the packet rather than zeroed.
        List<PositionElement> relatives = playerPositionPacket.getRelatives();
        Vector3d current = MovementSync.INSTANCE.position.get();
        float currentYaw = MovementSync.INSTANCE.yaw.get();
        float currentPitch = MovementSync.INSTANCE.pitch.get();
        Vector3d position = new Vector3d(
                (relatives.contains(PositionElement.X) ? current.x : 0) + playerPositionPacket.getPosition().getX(),
                (relatives.contains(PositionElement.Y) ? current.y : 0) + playerPositionPacket.getPosition().getY(),
                (relatives.contains(PositionElement.Z) ? current.z : 0) + playerPositionPacket.getPosition().getZ());
        float yaw = (relatives.contains(PositionElement.Y_ROT) ? currentYaw : 0) + playerPositionPacket.getYRot();
        float pitch = Math.max(-90f, Math.min(90f,
                (relatives.contains(PositionElement.X_ROT) ? currentPitch : 0) + playerPositionPacket.getXRot()));
        Vector3d velocity = new Vector3d(MovementSync.INSTANCE.velocity.get());
        if (relatives.contains(PositionElement.ROTATE_DELTA)) {
            double pitchDelta = Math.toRadians(currentPitch - pitch);
            double yawDelta = Math.toRadians(currentYaw - yaw);
            velocity.set(velocity.x,
                    velocity.y * Math.cos(pitchDelta) + velocity.z * Math.sin(pitchDelta),
                    velocity.z * Math.cos(pitchDelta) - velocity.y * Math.sin(pitchDelta));
            velocity.set(velocity.x * Math.cos(yawDelta) + velocity.z * Math.sin(yawDelta),
                    velocity.y,
                    velocity.z * Math.cos(yawDelta) - velocity.x * Math.sin(yawDelta));
        }
        velocity.set(
                (relatives.contains(PositionElement.DELTA_X) ? velocity.x : 0) + playerPositionPacket.getDeltaMovement().getX(),
                (relatives.contains(PositionElement.DELTA_Y) ? velocity.y : 0) + playerPositionPacket.getDeltaMovement().getY(),
                (relatives.contains(PositionElement.DELTA_Z) ? velocity.z : 0) + playerPositionPacket.getDeltaMovement().getZ());
        TeleportEvent teleportEvent = new TeleportEvent(playerPositionPacket.getId(), position);
        Bot.INSTANCE.getPluginManager().events().callEvent(teleportEvent);
        if (teleportEvent.isDefaultActionCancelled()) return;
        MovementSync.INSTANCE.position.set(position);
        MovementSync.INSTANCE.pitch.set(pitch);
        MovementSync.INSTANCE.yaw.set(yaw);
        session.send(new ServerboundAcceptTeleportationPacket(playerPositionPacket.getId()));
        // The vanilla client follows the confirmation with a full position packet
        // echoing the teleport target; servers treat the teleport as pending until then.
        session.send(new ServerboundMovePlayerPosRotPacket(
                false, false, position.x, position.y, position.z,
                yaw, pitch));
        MovementSync.INSTANCE.cancelNavigation();
        // A teleport does not end a glide: the server keeps its gliding flag set and
        // sends no metadata update, so clearing it here would leave the two sides out of sync.
        MovementSync.INSTANCE.velocity.set(velocity);
        checkOnGround();
    }
}
