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
    public void onEntityPositionSync(ReceivePacketEvent<ClientboundEntityPositionSyncPacket> event) {
        var packet = event.getPacket();
        var entity = MovementSync.INSTANCE.getWorld().getEntity(packet.getId());
        if (entity == null) return;
        var p = packet.getPosition();
        entity.moveTo(new org.joml.Vector3d(p.getX(), p.getY(), p.getZ()));
        var v = packet.getDeltaMovement();
        entity.setMovement(new org.joml.Vector3d(v.getX(), v.getY(), v.getZ()));
        entity.setYaw(packet.getYRot());
        entity.setPitch(packet.getXRot());
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
        var sync = MovementSync.INSTANCE;
        for (int id : receivePacketEvent.getPacket().getEntityIds()) {
            if (id != sync.getFollowTargetId()) continue;
            var target = sync.getWorld().getEntity(id);
            MovementSync.getLogger().info("Follow target removed by server: entity={}, lastPosition={}, observer={}, distance={}; tracking-range removal versus despawn is not specified by packet",
                    id, target == null ? null : target.getPosition(), sync.position.get(),
                    target == null ? null : target.getPosition().distance(sync.position.get()));
        }
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
                    boolean before = MovementSync.INSTANCE.isGliding();
                    MovementSync.INSTANCE.applySharedFlags(flags);
                    if (before != MovementSync.INSTANCE.isGliding()) MovementSync.getLogger().info(
                            "Authoritative glide metadata: rawFlags={}, before={}, after={}, position={}, velocity={}, lastPhysicsTick={}",
                            flags, before, MovementSync.INSTANCE.isGliding(), MovementSync.INSTANCE.position.get(),
                            MovementSync.INSTANCE.velocity.get(), MovementSync.INSTANCE.getLastFlightTick());
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
                        if ("shared_flags".equals(key) && value instanceof Byte flags) {
                            value = MovementSync.INSTANCE.applySharedFlags(flags);
                        }
                        if (value == null) {
                            MovementSync.INSTANCE.getSelfMetadata().remove(key);
                        } else {
                            MovementSync.INSTANCE.getSelfMetadata().put(key, value);
                        }
                    }
                }
            }
        }
    }
}
