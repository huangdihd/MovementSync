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
        MovementSync.INSTANCE.getWorld().handleAddEntityPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnSetPassengers(ReceivePacketEvent<ClientboundSetPassengersPacket> event) {
        ClientboundSetPassengersPacket packet = event.getPacket();
        boolean isPassenger = false;
        for (int passengerId : packet.getPassengerIds()) {
            if (passengerId == MovementSync.INSTANCE.entityId) {
                isPassenger = true;
                break;
            }
        }

        if (isPassenger) {
            MovementSync.INSTANCE.setVehicleId(packet.getEntityId());
        } else if (MovementSync.INSTANCE.getVehicleId() == packet.getEntityId()) {
            // If we were riding this entity but are no longer in the passenger list
            MovementSync.INSTANCE.setVehicleId(-1);
        }
    }

    @EventHandler
    public void OnMoveEntityPosition(ReceivePacketEvent<ClientboundMoveEntityPosPacket> receivePacketEvent) {
        MovementSync.INSTANCE.getWorld().handleMoveEntityPosPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnMoveEntityRotation(ReceivePacketEvent<ClientboundMoveEntityRotPacket> receivePacketEvent) {
        MovementSync.INSTANCE.getWorld().handleMoveEntityRotPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnMoveEntityPositronRotation(ReceivePacketEvent<ClientboundMoveEntityPosRotPacket> receivePacketEvent) {
        MovementSync.INSTANCE.getWorld().handleMoveEntityPosRotPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnMoveEntityRotateHead(ReceivePacketEvent<ClientboundRotateHeadPacket> receivePacketEvent) {
        MovementSync.INSTANCE.getWorld().handleRotateHeadPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnRemoveEntity(ReceivePacketEvent<ClientboundRemoveEntitiesPacket> receivePacketEvent) {
        MovementSync.INSTANCE.getWorld().handleRemoveEntitiesPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnSetEntityMetadata(ReceivePacketEvent<ClientboundSetEntityDataPacket> receivePacketEvent) {
        ClientboundSetEntityDataPacket packet = receivePacketEvent.getPacket();
        MovementSync.INSTANCE.getWorld().handleSetEntityMetadataPacket(packet);

        if (packet.getEntityId() == MovementSync.INSTANCE.entityId) {
            // Entity shared flags are protocol metadata index zero, even before registry loading.
            for (var entry : packet.getMetadata()) {
                if (entry.getId() == 0 && entry.getValue() instanceof Byte flags) {
                    boolean gliding = (flags & 0x80) != 0;
                    if (gliding && !MovementSync.INSTANCE.isGliding()) {
                        MovementSync.getLogger().info("Server metadata: adopting authoritative gliding state");
                    }
                    MovementSync.INSTANCE.setGliding(gliding);
                }
            }
            EntityRegistry.EntityEntry registryEntry = EntityRegistry.Instance.getEntity("PLAYER");
            if (registryEntry != null && registryEntry.getMetadataKeys() != null) {
                List<String> keys = registryEntry.getMetadataKeys();
                for (org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.EntityMetadata<?, ?> entry : packet.getMetadata()) {
                    int id = entry.getId();
                    if (id >= 0 && id < keys.size()) {
                        String key = keys.get(id);
                        Object value = entry.getValue();
                        if (value == null) {
                            MovementSync.INSTANCE.getSelfMetadata().remove(key);
                        } else {
                            MovementSync.INSTANCE.getSelfMetadata().put(key, value);
                        }
                        if ("shared_flags".equals(key) && value instanceof Byte flags) {
                            MovementSync.INSTANCE.setGliding((flags & 0x80) != 0);
                        }
                    }
                }
            }
        }
    }
}
