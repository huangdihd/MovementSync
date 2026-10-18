package xin.bbtt.listeners;

import net.kyori.adventure.key.Key;
import org.geysermc.mcprotocollib.network.ClientSession;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.EntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.MetadataTypes;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.type.ByteEntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.GameMode;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerSpawnInfo;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundRespawnPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundSetEntityDataPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundSetHealthPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.inventory.ItemRegistry;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.events.ReceivePacketEvent;

import java.lang.reflect.Proxy;

import static org.junit.jupiter.api.Assertions.*;

class DeathRespawnFlightLifecycleTest {
    private final MovementSync previous = MovementSync.INSTANCE;
    private final MovementSync sync = new MovementSync();
    private final MovementParametersPacketListener parameters = new MovementParametersPacketListener();

    DeathRespawnFlightLifecycleTest() {
        sync.entityId = 42;
    }

    @AfterEach
    void restore() {
        MovementSync.INSTANCE = previous;
    }

    private void flags(int flags) {
        var packet = new ClientboundSetEntityDataPacket(42, new EntityMetadata<?, ?>[]{
                new ByteEntityMetadata(0, MetadataTypes.BYTE, (byte) flags)});
        new EntityPacketListener().OnSetEntityMetadata(new ReceivePacketEvent<>(packet, ClientboundSetEntityDataPacket.class));
    }

    private void respawn() {
        var spawn = new PlayerSpawnInfo(0, Key.key("minecraft:overworld"), 0, GameMode.SURVIVAL,
                null, false, true, null, 0, 63);
        var packet = new ClientboundRespawnPacket(spawn, false, false);
        new LoginPacketListener().packetReceived(null, packet);
        parameters.packetReceived(null, packet);
    }

    @Test
    void lavaDeathAndRespawnCannotAdoptOldFlagsOrRestartSafeLanding() {
        sync.setRecoveryDimension("minecraft:the_nether");
        flags(0x81);
        assertTrue(sync.isGliding());
        sync.velocity.set(new Vector3d(2, -1, 3));
        sync.fallDistance.set(25.0);
        sync.jump();
        parameters.packetReceived(null, new ClientboundSetHealthPacket(0, 20, 5));
        assertFalse(sync.isGliding());
        assertEquals(new Vector3d(), sync.velocity.get());
        assertEquals(0.0, sync.fallDistance.get());
        assertFalse(sync.snapshotInput().jump());
        flags(0x81);
        assertFalse(sync.isGliding(), "late pre-respawn metadata");
        respawn();
        parameters.packetReceived(null, new ClientboundSetHealthPacket(20, 20, 5));
        assertEquals("minecraft:overworld", sync.getRecoveryDimension());
        for (int i = 0; i < 3; i++) {
            flags(0); // A clearing packet alone does not prove a new flight.
            flags(0x81);
            assertFalse(sync.isGliding());
            assertEquals((byte) 1, sync.getSelfMetadata().get("shared_flags"));
            sync.cancelNavigation();
            assertNull(sync.getMovementController().getCurrentMovement());
            assertTrue(sync.getMovementController().getMovements().isEmpty());
        }
    }

    @Test
    void respawnWithoutDeathPacketAlsoInvalidatesFlightButReconnectCanAdoptIt() {
        flags(0x80);
        respawn();
        flags(0x80);
        assertFalse(sync.isGliding());
        sync.resetFlightMetadataSession();
        flags(0x80);
        assertTrue(sync.isGliding(), "new sessions may reconnect airborne");
        flags(0);
        assertFalse(sync.isGliding(), "authoritative stop remains effective");
    }

    @Test
    void onlyValidNewDeploymentRearmsMetadataAndOldQueuedActionsAreDiscarded() throws Exception {
        var sessionField = Bot.class.getDeclaredField("session");
        sessionField.setAccessible(true);
        Object previousSession = sessionField.get(Bot.INSTANCE);
        var session = Proxy.newProxyInstance(getClass().getClassLoader(), new Class<?>[]{ClientSession.class},
                (proxy, method, args) -> method.getName().equals("isConnected") ? true : null);
        sessionField.set(Bot.INSTANCE, session);
        try {
            var inventory = new ItemStack[46];
            inventory[6] = new ItemStack(ItemRegistry.Instance.getItem("elytra").getId(), 1);
            sync.inventoryManager.setContainerItems(0, inventory);
            sync.onGround.set(false);
            assertTrue(sync.startGliding());
            sync.onPlayerDeath();
            assertNull(sync.pollBeforeMovementPacket());
            assertFalse(sync.startGliding(), "dead players cannot deploy");
            respawn();
            sync.onGround.set(true);
            assertFalse(sync.startGliding());
            flags(0x80);
            assertFalse(sync.isGliding(), "failed start must not rearm stale metadata");
            // Respawn clears the inventory session; await a fresh server inventory.
            sync.inventoryManager.setContainerItems(0, inventory);
            sync.onGround.set(false);
            assertTrue(sync.startGliding());
            assertInstanceOf(ServerboundPlayerCommandPacket.class, sync.pollBeforeMovementPacket());
            flags(0x80);
            assertTrue(sync.isGliding());
            flags(0);
            assertFalse(sync.isGliding());
        } finally {
            sessionField.set(Bot.INSTANCE, previousSession);
        }
    }
}
