package xin.bbtt;

import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.EntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.MetadataTypes;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.type.ByteEntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.type.ObjectEntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundRemoveEntitiesPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundSetEntityDataPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Entity.Entity;
import xin.bbtt.Entity.EntityRegistry;
import xin.bbtt.listeners.EntityPacketListener;
import xin.bbtt.listeners.ServerChangeListener;
import xin.bbtt.mcbot.events.ReceivePacketEvent;

import java.util.OptionalInt;
import java.util.UUID;

import static org.junit.jupiter.api.Assertions.*;

final class ElytraStateTest {
    private final MovementSync movementSync = new MovementSync();
    private final EntityPacketListener listener = new EntityPacketListener();

    ElytraStateTest() {
        movementSync.entityId = 42;
    }

    @AfterEach
    void resetSingleton() {
        MovementSync.INSTANCE = null;
    }

    @Test
    void requestsUseExplicitStartAndHeldRocketWithoutInventingAttachment() throws Exception {
        java.lang.reflect.Field sessionField = xin.bbtt.mcbot.Bot.class.getDeclaredField("session");
        sessionField.setAccessible(true);
        Object previousSession = sessionField.get(xin.bbtt.mcbot.Bot.INSTANCE);
        java.util.List<Object> packets = new java.util.ArrayList<>();
        Object session = java.lang.reflect.Proxy.newProxyInstance(getClass().getClassLoader(),
                new Class<?>[]{org.geysermc.mcprotocollib.network.ClientSession.class}, (proxy, method, arguments) -> {
                    if (method.getName().equals("isConnected")) return true;
                    if (method.getName().equals("send")) packets.add(arguments[0]);
                    return null;
                });
        sessionField.set(xin.bbtt.mcbot.Bot.INSTANCE, session);
        try {
            var inventory = new org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack[46];
            inventory[6] = new org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack(
                    xin.bbtt.inventory.ItemRegistry.Instance.getItem("elytra").getId(), 1);
            inventory[36] = new org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack(
                    xin.bbtt.inventory.ItemRegistry.Instance.getItem("firework_rocket").getId(), 3);
            movementSync.inventoryManager.setContainerItems(0, inventory);
            assertFalse(movementSync.startGliding(), "grounded");
            assertTrue(packets.isEmpty());
            movementSync.onGround.set(false);
            movementSync.setVehicleId(99);
            assertFalse(movementSync.startGliding(), "riding");
            assertTrue(packets.isEmpty());
            movementSync.setVehicleId(-1);
            var elytra = inventory[6];
            inventory[6] = null;
            assertFalse(movementSync.startGliding(), "no chest elytra");
            assertTrue(packets.isEmpty());
            inventory[6] = elytra;
            assertTrue(movementSync.startGliding());
            assertTrue(movementSync.isGliding());
            assertTrue(movementSync.snapshotInput().jump(), "deployment tick presses jump");
            assertFalse(movementSync.snapshotInput().jump(), "deployment jump is one-shot");
            var start = assertInstanceOf(org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket.class, movementSync.pollBeforeMovementPacket());
            assertEquals(org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerState.START_ELYTRA_FLYING, start.getState());
            assertEquals(42, start.getEntityId());
            assertTrue(packets.isEmpty(), "start is queued until the movement tick");
            assertFalse(movementSync.startGliding());
            assertNull(movementSync.pollBeforeMovementPacket(), "duplicate start must not queue another action");
            assertTrue(movementSync.useFirework());
            var use = assertInstanceOf(org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket.class, movementSync.pollBeforeMovementPacket());
            assertEquals(org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand.MAIN_HAND, use.getHand());
            assertFalse(movementSync.hasAttachedFirework());
            assertEquals(3, inventory[36].getAmount());
            inventory[40] = inventory[36];
            inventory[36] = null;
            assertTrue(movementSync.useFirework());
            var switchSlot = assertInstanceOf(org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSetCarriedItemPacket.class, movementSync.pollBeforeMovementPacket());
            assertEquals(4, switchSlot.getSlot());
            var hotbar = assertInstanceOf(org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket.class, movementSync.pollBeforeMovementPacket());
            assertEquals(org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand.MAIN_HAND, hotbar.getHand());
            assertTrue(movementSync.stopGliding());
            assertTrue(packets.isEmpty(), "actions are queued; stop must not send a start/toggle packet");
            movementSync.setGliding(true);
            addAttachedFirework(7, OptionalInt.of(42));
            new xin.bbtt.listeners.TeleportPacketListener().packetReceived(
                    (org.geysermc.mcprotocollib.network.Session) session,
                    new org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundPlayerPositionPacket(
                            1, 0, 64, 0, 0, 0, 0, 0, 0));
            assertTrue(movementSync.isGliding(), "a teleport must not end the glide");
            assertTrue(movementSync.hasAttachedFirework());
            movementSync.setGliding(true);
            addAttachedFirework(8, OptionalInt.of(42));
            new xin.bbtt.listeners.RespawnPacketListener().packetReceived(
                    (org.geysermc.mcprotocollib.network.Session) session,
                    new org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundPlayerCombatKillPacket(
                            42, net.kyori.adventure.text.Component.text("test death")));
            assertFalse(movementSync.isGliding());
            assertFalse(movementSync.hasAttachedFirework());
        } finally {
            sessionField.set(xin.bbtt.mcbot.Bot.INSTANCE, previousSession);
        }
    }

    @Test
    void stopIsLocalAndIdempotent() {
        assertFalse(movementSync.isGliding());
        movementSync.setGliding(true);
        addAttachedFirework(7, OptionalInt.of(42));
        assertTrue(movementSync.stopGliding());
        assertFalse(movementSync.isGliding());
        assertFalse(movementSync.hasAttachedFirework());
        assertFalse(movementSync.stopGliding());
    }

    @Test
    void startAndUseRejectInvalidLocalPreconditions() {
        assertFalse(movementSync.startGliding());
        movementSync.onGround.set(false);
        assertFalse(movementSync.startGliding(), "missing chest elytra");
        assertFalse(movementSync.useFirework());
        movementSync.setGliding(true);
        assertFalse(movementSync.useFirework(), "missing held rocket");
        assertFalse(movementSync.hasAttachedFirework());
    }

    @Test
    void serverSharedFlagsCorrectLocalStateWithoutPoseFallback() {
        metadata(42, new ByteEntityMetadata(0, MetadataTypes.BYTE, (byte) 0x80));
        assertTrue(movementSync.isGliding());
        metadata(42, new ByteEntityMetadata(1, MetadataTypes.BYTE, (byte) 0));
        assertTrue(movementSync.isGliding(), "unrelated metadata must preserve gliding");
        metadata(43, new ByteEntityMetadata(0, MetadataTypes.BYTE, (byte) 0));
        assertTrue(movementSync.isGliding(), "another entity cannot correct self");
        metadata(42, new ByteEntityMetadata(0, MetadataTypes.BYTE, (byte) 0x02));
        assertFalse(movementSync.isGliding());
    }

    @Test
    void invalidSelfMetadataIndexIsIgnored() {
        metadata(42, new ByteEntityMetadata(-1, MetadataTypes.BYTE, (byte) 0));
        metadata(42, new ByteEntityMetadata(999, MetadataTypes.BYTE, (byte) 0));
        assertTrue(movementSync.getSelfMetadata().isEmpty());
    }

    @Test
    void onlyRocketAttachmentToSelfCountsAndMultipleRocketsSurviveOneRemoval() {
        movementSync.setGliding(true);
        addEntity(7, EntityType.FIREWORK_ROCKET);
        addEntity(8, EntityType.FIREWORK_ROCKET);
        addEntity(9, EntityType.PLAYER);
        attachment(9, OptionalInt.of(42));
        assertFalse(movementSync.hasAttachedFirework());
        attachment(7, OptionalInt.of(43));
        assertFalse(movementSync.hasAttachedFirework());
        attachment(7, OptionalInt.of(42));
        attachment(8, OptionalInt.of(42));
        assertTrue(movementSync.hasAttachedFirework());
        ClientboundRemoveEntitiesPacket removal = new ClientboundRemoveEntitiesPacket(new int[]{7});
        listener.OnRemoveEntity(new ReceivePacketEvent<>(removal, ClientboundRemoveEntitiesPacket.class));
        assertTrue(movementSync.hasAttachedFirework());
        attachment(8, OptionalInt.empty());
        assertFalse(movementSync.hasAttachedFirework());
    }

    @Test
    void clearAndServerChangeDiscardFlightState() {
        movementSync.setGliding(true);
        addAttachedFirework(7, OptionalInt.of(42));
        movementSync.getSelfMetadata().put("shared_flags", (byte) 0x80);
        movementSync.clearFlightState();
        assertFalse(movementSync.isGliding());
        assertFalse(movementSync.hasAttachedFirework());
        assertFalse(movementSync.getSelfMetadata().containsKey("shared_flags"));
        movementSync.setGliding(true);
        addAttachedFirework(8, OptionalInt.of(42));
        new ServerChangeListener().onServerChange(null);
        assertFalse(movementSync.isGliding());
        assertFalse(movementSync.hasAttachedFirework());
    }

    private void addAttachedFirework(int entityId, OptionalInt target) {
        addEntity(entityId, EntityType.FIREWORK_ROCKET);
        attachment(entityId, target);
    }

    private void addEntity(int entityId, EntityType type) {
        movementSync.world.getEntities().put(entityId, new Entity(entityId, UUID.randomUUID(), type,
                0, 64, 0, 0, 0, 0, new Vector3d()));
    }

    private void attachment(int entityId, OptionalInt target) {
        int index = EntityRegistry.Instance.getEntity("FIREWORK_ROCKET").getMetadataKeys().indexOf("attached_to_target");
        assertEquals(9, index);
        metadata(entityId, new ObjectEntityMetadata<>(index, MetadataTypes.OPTIONAL_UNSIGNED_INT, target));
    }

    private void metadata(int entityId, EntityMetadata<?, ?> entry) {
        ClientboundSetEntityDataPacket packet = new ClientboundSetEntityDataPacket(entityId, new EntityMetadata<?, ?>[]{entry});
        listener.OnSetEntityMetadata(new ReceivePacketEvent<>(packet, ClientboundSetEntityDataPacket.class));
    }
}
