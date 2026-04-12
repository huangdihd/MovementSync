package xin.bbtt.Entity;

import lombok.Data;
import lombok.NonNull;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundAddEntityPacket;
import org.jetbrains.annotations.NotNull;
import org.joml.Vector3d;

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

    public Entity(int entityId, @NotNull UUID uuid, @NotNull EntityType type, double x, double y, double z, float yaw, float headYaw, float pitch, Vector3d movement) {
        this.entityId = entityId;
        this.uuid = uuid;
        this.type = type;
        this.position = new Vector3d(x, y, z);
        this.yaw = yaw;
        this.headYaw = headYaw;
        this.pitch = pitch;
        this.movement = movement;
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
