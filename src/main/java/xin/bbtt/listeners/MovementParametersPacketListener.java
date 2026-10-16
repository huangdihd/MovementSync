package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.DisconnectedEvent;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundLoginPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundRespawnPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundRemoveMobEffectPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundUpdateAttributesPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundUpdateMobEffectPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundPlayerAbilitiesPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundSetHealthPacket;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.MovementParameterState;

/** Only self packets may update the movement snapshot. */
public final class MovementParametersPacketListener extends SessionAdapter {
    @Override
    public void packetReceived(Session session, Packet packet) {
        MovementSync movementSync = MovementSync.INSTANCE;
        MovementParameterState state = movementSync.getMovementParameterState();
        if (packet instanceof ClientboundLoginPacket) {
            state.reset();
        } else if (packet instanceof ClientboundRespawnPacket respawn) {
            state.respawn(respawn.isKeepAttributeModifiers());
        } else if (packet instanceof ClientboundUpdateAttributesPacket attributes) {
            if (attributes.getEntityId() != movementSync.entityId || movementSync.entityId < 0) return;
            state.updateAttributes(attributes.getAttributes());
        } else if (packet instanceof ClientboundUpdateMobEffectPacket effect) {
            if (effect.getEntityId() != movementSync.entityId || movementSync.entityId < 0) return;
            state.updateEffect(effect.getEffect(), effect.getAmplifier());
        } else if (packet instanceof ClientboundRemoveMobEffectPacket effect) {
            if (effect.getEntityId() != movementSync.entityId || movementSync.entityId < 0) return;
            state.removeEffect(effect.getEffect());
        } else if (packet instanceof ClientboundSetHealthPacket health) {
            state.updateFood(health.getFood());
            if (health.getHealth() <= 0) {
                movementSync.clearFlightState();
                movementSync.cancelNavigation();
            }
        } else if (packet instanceof ClientboundPlayerAbilitiesPacket abilities) {
            state.updateAbilities(abilities.isCanFly());
        }
    }

    @Override
    public void disconnected(DisconnectedEvent event) {
        MovementSync.INSTANCE.suspendNavigationForRecovery();
        MovementSync.INSTANCE.getMovementParameterState().reset();
    }
}
