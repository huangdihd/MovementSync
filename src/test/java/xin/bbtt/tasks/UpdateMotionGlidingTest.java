package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalInt;
import java.util.UUID;
import org.geysermc.mcprotocollib.network.ClientSession;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.EntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.MetadataTypes;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.type.ObjectEntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundRemoveEntitiesPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundSetEntityDataPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Entity.Entity;
import xin.bbtt.Entity.EntityRegistry;
import xin.bbtt.MovementSync;
import xin.bbtt.listeners.EntityPacketListener;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;
import xin.bbtt.mcbot.events.ReceivePacketEvent;
import xin.bbtt.world.World;

final class UpdateMotionGlidingTest {
    private static final double BASE_Y = -0.018 * (double) 0.98f;
    private static final double BASE_Z = 0.0018 * (double) 0.99f;
    private static final int FIREWORK_ID = 42;
    private final List<Object> sentPackets = new ArrayList<>();
    private final EntityPacketListener entityListener = new EntityPacketListener();
    private MovementSync movementSync;
    private updateMotionTask task;

    @BeforeEach
    void setUp() throws Exception {
        movementSync = new MovementSync();
        movementSync.onRecoveryTeleport();
        movementSync.entityId = 7;
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }

            @Override
            public int getBlockAt(Vector3d position) {
                return Math.floor(position.y) == 0 ? 1 : 0;
            }
        };
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        setBotField("running", true);
        Bot.INSTANCE.setServer(Server.Game);
        setBotField("session", Proxy.newProxyInstance(
            ClientSession.class.getClassLoader(), new Class<?>[]{ClientSession.class},
            (proxy, method, arguments) -> {
                if (method.getName().equals("isConnected")) return true;
                // The per-tick end marker is asserted separately; keep it out of movement-packet counts.
                if (method.getName().equals("send") && !(arguments[0] instanceof org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.ServerboundClientTickEndPacket)) sentPackets.add(arguments[0]);
                Class<?> returnType = method.getReturnType();
                if (!returnType.isPrimitive()) return null;
                if (returnType == boolean.class) return false;
                if (returnType == byte.class) return (byte) 0;
                if (returnType == short.class) return (short) 0;
                if (returnType == int.class) return 0;
                if (returnType == long.class) return 0L;
                if (returnType == float.class) return 0.0f;
                if (returnType == double.class) return 0.0d;
                if (returnType == char.class) return '\0';
                return null;
            }));
        resetAirborneState();
    }

    @AfterEach
    void tearDown() throws Exception {
        setBotField("running", false);
        Bot.INSTANCE.setServer(null);
        setBotField("session", null);
        MovementSync.INSTANCE = null;
    }

    @Test
    void disconnectedSessionSkipsPositionAndTickEnd() throws Exception {
        setBotField("session", null);
        org.junit.jupiter.api.Assertions.assertDoesNotThrow(() -> task.syncPositionToServer(false, true, true));
        org.junit.jupiter.api.Assertions.assertDoesNotThrow(task::run);
        assertTrue(sentPackets.isEmpty());
    }

    @Test
    void stationarySouthFacingGlideUsesFlightTravelAndSendsPosition() {
        task.run();

        assertVector(movementSync.velocity.get(), 0, BASE_Y, BASE_Z);
        assertTravelPosition();
        assertTrue(movementSync.isGliding());
        assertFalse(movementSync.onGround.get());
        assertFalse(movementSync.hasAttachedFirework());
    }

    @Test
    void landingRetainsGlidingAndRocketUntilServerMetadataClearsFlag() {
        attachFirework(movementSync.entityId);
        movementSync.position.set(new Vector3d(0.5, 1.01, 0.5));

        task.run();

        assertVector(movementSync.position.get(), 0.5, 1, 0.5 + BASE_Z);
        assertVector(movementSync.velocity.get(), 0, 0, 0.85 + BASE_Z / 2);
        assertTrue(movementSync.onGround.get());
        assertTrue(movementSync.isGliding(), "contact must not clear client flight");
        assertTrue(positionPacket().isOnGround());
        for (int tick = 0; tick < 5; tick++) {
            task.run();
            assertTrue(movementSync.isGliding());
            assertTrue(movementSync.hasAttachedFirework());
        }
        int flagsId = EntityRegistry.Instance.getEntity("PLAYER").getMetadataKeys().indexOf("shared_flags");
        var metadata = new ClientboundSetEntityDataPacket(movementSync.entityId, new EntityMetadata<?, ?>[]{
            new org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.type.ByteEntityMetadata(
                    flagsId, MetadataTypes.BYTE, (byte) 0)
        });
        entityListener.OnSetEntityMetadata(new ReceivePacketEvent<>(metadata, ClientboundSetEntityDataPacket.class));
        assertFalse(movementSync.isGliding());
        assertFalse(movementSync.hasAttachedFirework());
    }

    @Test
    void attachedOwnFireworkBoostsVelocityAfterMovementAndRemovalStopsBoost() {
        attachFirework(movementSync.entityId);
        assertTrue(movementSync.hasAttachedFirework());

        task.run();

        assertVector(movementSync.velocity.get(), 0, BASE_Y / 2, 0.85 + BASE_Z / 2);
        assertTravelPosition();
        entityListener.OnRemoveEntity(new ReceivePacketEvent<>(
            new ClientboundRemoveEntitiesPacket(new int[]{FIREWORK_ID}),
            ClientboundRemoveEntitiesPacket.class));
        assertFalse(movementSync.hasAttachedFirework());
        assertFalse(movementSync.getWorld().getEntities().containsKey(FIREWORK_ID));

        // Reset only kinematics so both ticks have identical travel inputs.
        resetAirborneState();
        task.run();

        assertVector(movementSync.velocity.get(), 0, BASE_Y, BASE_Z);
        assertTravelPosition();
    }

    @Test
    void fireworkAttachedToAnotherEntityDoesNotBoost() {
        attachFirework(movementSync.entityId + 1);
        assertFalse(movementSync.hasAttachedFirework());

        task.run();

        assertVector(movementSync.velocity.get(), 0, BASE_Y, BASE_Z);
        assertTravelPosition();
    }

    private void attachFirework(int targetEntityId) {
        Entity firework = new Entity(FIREWORK_ID, UUID.randomUUID(), EntityType.FIREWORK_ROCKET,
            0.5, 10, 0.5, 0, 0, 0, new Vector3d());
        movementSync.getWorld().getEntities().put(FIREWORK_ID, firework);
        int attachmentId = EntityRegistry.Instance.getEntity("FIREWORK_ROCKET")
            .getMetadataKeys().indexOf("attached_to_target");
        assertTrue(attachmentId >= 0);
        var packet = new ClientboundSetEntityDataPacket(FIREWORK_ID, new EntityMetadata<?, ?>[]{
            new ObjectEntityMetadata<>(attachmentId, MetadataTypes.OPTIONAL_UNSIGNED_INT,
                OptionalInt.of(targetEntityId))
        });
        entityListener.OnSetEntityMetadata(new ReceivePacketEvent<>(packet, ClientboundSetEntityDataPacket.class));
    }

    private void resetAirborneState() {
        movementSync.position.set(new Vector3d(0.5, 10, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.yaw.set(0f);
        movementSync.pitch.set(0f);
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        sentPackets.clear();
        task = new updateMotionTask();
    }

    private void assertTravelPosition() {
        assertVector(movementSync.position.get(), 0.5, 10 + BASE_Y, 0.5 + BASE_Z);
        ServerboundMovePlayerPosPacket packet = positionPacket();
        assertEquals(0.5, packet.getX(), 1.0e-9);
        assertEquals(10 + BASE_Y, packet.getY(), 1.0e-9);
        assertEquals(0.5 + BASE_Z, packet.getZ(), 1.0e-9);
        assertFalse(packet.isOnGround());
    }

    private ServerboundMovePlayerPosPacket positionPacket() {
        assertEquals(1, sentPackets.size());
        return assertInstanceOf(ServerboundMovePlayerPosPacket.class, sentPackets.get(0));
    }

    private static void assertVector(Vector3d actual, double x, double y, double z) {
        assertEquals(x, actual.x, 1.0e-9);
        assertEquals(y, actual.y, 1.0e-9);
        assertEquals(z, actual.z, 1.0e-9);
    }

    private static void setBotField(String name, Object value) throws Exception {
        Field field = Bot.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(Bot.INSTANCE, value);
    }
}
