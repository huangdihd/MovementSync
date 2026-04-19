package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.*;
import xin.bbtt.MovementSync;
import xin.bbtt.Entity.EntityRegistry;
import xin.bbtt.mcbot.event.EventHandler;
import xin.bbtt.mcbot.event.Listener;
import xin.bbtt.mcbot.events.ReceivePacketEvent;

import java.util.List;

public class EntityPacketListener implements Listener {

    @EventHandler
    public void OnAddEntities(ReceivePacketEvent<ClientboundAddEntityPacket> receivePacketEvent) {
        MovementSync.Instance.getWorld().handleAddEntityPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnSetPassengers(ReceivePacketEvent<ClientboundSetPassengersPacket> event) {
        ClientboundSetPassengersPacket packet = event.getPacket();
        boolean isPassenger = false;
        for (int passengerId : packet.getPassengerIds()) {
            if (passengerId == MovementSync.Instance.entityId) {
                isPassenger = true;
                break;
            }
        }

        if (isPassenger) {
            MovementSync.Instance.setVehicleId(packet.getEntityId());
        } else if (MovementSync.Instance.getVehicleId() == packet.getEntityId()) {
            // If we were riding this entity but are no longer in the passenger list
            MovementSync.Instance.setVehicleId(-1);
        }
    }

    @EventHandler
    public void OnMoveEntityPosition(ReceivePacketEvent<ClientboundMoveEntityPosPacket> receivePacketEvent) {
        MovementSync.Instance.getWorld().handleMoveEntityPosPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnMoveEntityRotation(ReceivePacketEvent<ClientboundMoveEntityRotPacket> receivePacketEvent) {
        MovementSync.Instance.getWorld().handleMoveEntityRotPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnMoveEntityPositronRotation(ReceivePacketEvent<ClientboundMoveEntityPosRotPacket> receivePacketEvent) {
        MovementSync.Instance.getWorld().handleMoveEntityPosRotPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnMoveEntityRotateHead(ReceivePacketEvent<ClientboundRotateHeadPacket> receivePacketEvent) {
        MovementSync.Instance.getWorld().handleRotateHeadPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnRemoveEntity(ReceivePacketEvent<ClientboundRemoveEntitiesPacket> receivePacketEvent) {
        MovementSync.Instance.getWorld().handleRemoveEntitiesPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnSetEntityMetadata(ReceivePacketEvent<ClientboundSetEntityDataPacket> receivePacketEvent) {
        ClientboundSetEntityDataPacket packet = receivePacketEvent.getPacket();
        MovementSync.Instance.getWorld().handleSetEntityMetadataPacket(packet);

        if (packet.getEntityId() == MovementSync.Instance.entityId) {
            EntityRegistry.EntityEntry registryEntry = EntityRegistry.Instance.getEntity("PLAYER");
            if (registryEntry != null && registryEntry.getMetadataKeys() != null) {
                List<String> keys = registryEntry.getMetadataKeys();
                for (org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.EntityMetadata<?, ?> entry : packet.getMetadata()) {
                    int id = entry.getId();
                    if (id < keys.size()) {
                        String key = keys.get(id);
                        Object value = entry.getValue();
                        MovementSync.Instance.getSelfMetadata().put(key, value);
                    }
                }
            }
        }
    }
}
