package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundSetEntityMotionPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundExplodePacket;
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
        MovementSync movementSync = MovementSync.INSTANCE;
        if (movementSync == null) return;
        if (packet instanceof ClientboundSetEntityMotionPacket motionPacket) {
            if (motionPacket.getEntityId() != movementSync.entityId) return;
            // Share the tick/Pong boundary: neither physics writeback may erase this packet.
            synchronized (movementSync) {
                movementSync.velocity.set(new Vector3d(
                        motionPacket.getMovement().getX(),
                        motionPacket.getMovement().getY(),
                        motionPacket.getMovement().getZ()));
            }
            return;
        }
        if (!(packet instanceof ClientboundExplodePacket explosionPacket)) return;
        var knockback = explosionPacket.getPlayerKnockback();
        if (knockback == null) return;
        synchronized (movementSync) {
            // Vanilla adds explosion impulse; retain current motion and never mutate its snapshot.
            movementSync.velocity.updateAndGet(velocity -> new Vector3d(velocity)
                    .add(knockback.getX(), knockback.getY(), knockback.getZ()));
        }
    }
}
