package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundSetEntityMotionPacket;
import org.joml.Vector3d;
import xin.bbtt.MovementSync;

/**
 * Applies velocity the server assigns to the player, like the vanilla client.
 * Knockback and anti-cheat setbacks both rely on it: a setback teleport is
 * followed by the velocity the server expects the client to continue with.
 */
public class VelocityPacketListener extends SessionAdapter {
    @Override
    public void packetReceived(Session session, Packet packet) {
        if (!(packet instanceof ClientboundSetEntityMotionPacket motionPacket)) return;
        MovementSync movementSync = MovementSync.INSTANCE;
        if (movementSync == null || motionPacket.getEntityId() != movementSync.entityId) return;
        // Never replace the velocity halfway through a physics tick.
        synchronized (movementSync) {
            movementSync.velocity.set(new Vector3d(
                    motionPacket.getMovement().getX(),
                    motionPacket.getMovement().getY(),
                    motionPacket.getMovement().getZ()));
        }
    }
}
