package xin.bbtt.Entity;

import lombok.Data;
import lombok.NonNull;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundAddEntityPacket;
import org.jetbrains.annotations.NotNull;
import org.joml.Vector3d;

import java.util.List;
import java.util.Map;
import java.util.UUID;

@Data
public class Entity {
    private int entityId;
    private final @NonNull UUID uuid;
    private final @NonNull EntityType type;
    private Vector3d position;
    private float yaw;
    private float headYaw;
    private float pitch;
    private Vector3d movement;
    private float width;
    private float height;
    private final Map<String, Object> metadata = new java.util.concurrent.ConcurrentHashMap<>();

    public Entity(int entityId, @NotNull UUID uuid, @NotNull EntityType type, double x, double y, double z, float yaw, float headYaw, float pitch, Vector3d movement) {
        this.entityId = entityId;
        this.uuid = uuid;
        this.type = type;
        this.position = new Vector3d(x, y, z);
        this.yaw = yaw;
        this.headYaw = headYaw;
        this.pitch = pitch;
        this.movement = movement;
        this.setSize(type);
    }

    public void updateMetadata(org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.EntityMetadata<?, ?>[] metadataEntries) {
        EntityRegistry.EntityEntry registryEntry = EntityRegistry.Instance.getEntity(type.name());
        if (registryEntry == null || registryEntry.getMetadataKeys() == null) return;

        List<String> keys = registryEntry.getMetadataKeys();
        for (org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.EntityMetadata<?, ?> entry : metadataEntries) {
            int id = entry.getId();
            if (id < keys.size()) {
                String key = keys.get(id);
                Object value = entry.getValue();
                this.metadata.put(key, value);
            }
        }
        // Metadata update might change size (e.g. pose change)
        this.recalculateSize();
    }

    private void recalculateSize() {
        // Here we can adjust height based on pose if present in metadata
        // For now, reset to base size and then apply modifications
        this.setSize(this.type);
        
        Object pose = this.metadata.get("pose");
        if (pose != null) {
            String poseName = pose.toString();
            if (poseName.contains("SNEAKING")) {
                this.height *= 0.8333333f; // 1.5/1.8
            } else if (poseName.contains("SWIMMING") || poseName.contains("FALL_FLYING")) {
                this.height = 0.6f;
            }
        }
    }

    private void setSize(EntityType type) {
        EntityRegistry.EntityEntry entry = EntityRegistry.Instance.getEntity(type.name());
        if (entry != null) {
            this.width = entry.getWidth();
            this.height = entry.getHeight();
        } else {
            // Default to player-like dimensions if not found in registry
            this.width = 0.6f;
            this.height = 1.8f;
        }
    }

    public Entity(int entityId, @NonNull UUID uuid, @NonNull EntityType type, double x, double y, double z, float yaw, float headYaw, float pitch, org.cloudburstmc.math.vector.Vector3d movement) {
        this(entityId, uuid, type, x, y, z, yaw, headYaw, pitch, new Vector3d(movement.getX(), movement.getY(), movement.getZ()));
    }

    public static Entity fromPacket(ClientboundAddEntityPacket packet) {
        return new Entity(packet.getEntityId(), packet.getUuid(), packet.getType(), packet.getX(), packet.getY(), packet.getZ(), packet.getYaw(), packet.getHeadYaw(), packet.getPitch(), packet.getMovement());
    }

    public void move(Vector3d delta){
        position.add(delta);
    }

    public void moveTo(Vector3d position) {
        this.position = position;
    }
}
