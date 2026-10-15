package xin.bbtt.listeners;

import net.kyori.adventure.key.Key;
import org.geysermc.mcprotocollib.protocol.data.game.entity.Effect;
import org.geysermc.mcprotocollib.protocol.data.game.entity.attribute.Attribute;
import org.geysermc.mcprotocollib.protocol.data.game.entity.attribute.AttributeModifier;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.GameMode;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerSpawnInfo;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundLoginPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundRespawnPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundRemoveMobEffectPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundUpdateAttributesPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundUpdateMobEffectPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundPlayerAbilitiesPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundSetHealthPacket;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.MovementParameters;

import java.util.List;

import static org.geysermc.mcprotocollib.protocol.data.game.entity.attribute.AttributeType.Builtin.*;
import static org.geysermc.mcprotocollib.protocol.data.game.entity.attribute.ModifierOperation.ADD;
import static org.junit.jupiter.api.Assertions.*;

class MovementParametersPacketListenerTest {
    private MovementSync previous;
    private MovementSync movementSync;
    private final MovementParametersPacketListener listener = new MovementParametersPacketListener();

    @BeforeEach
    void install() {
        previous = MovementSync.INSTANCE;
        movementSync = new MovementSync();
        movementSync.entityId = 42;
    }

    @AfterEach
    void restore() {
        MovementSync.INSTANCE = previous;
    }

    private void effect(int entityId, Effect effect, int amplifier, int duration) {
        listener.packetReceived(null, new ClientboundUpdateMobEffectPacket(
                entityId, effect, amplifier, duration, false, true, true, false));
    }

    private void attributes(int entityId) {
        listener.packetReceived(null, new ClientboundUpdateAttributesPacket(entityId, List.of(
                new Attribute(MOVEMENT_SPEED, 0.2, List.of(new AttributeModifier(Key.key("test:add"), 0.1, ADD))),
                new Attribute(STEP_HEIGHT, 1.5))));
    }

    private PlayerSpawnInfo spawn() {
        return new PlayerSpawnInfo(0, Key.key("minecraft:overworld"), 0, GameMode.SURVIVAL,
                null, false, true, null, 0, 63);
    }

    @Test
    void filtersOtherEntitiesAndReplacesAndRemovesOnlySelfEffects() {
        attributes(41);
        effect(41, Effect.JUMP_BOOST, 4, 100);
        assertEquals(MovementParameters.DEFAULT, movementSync.getMovementParameters());
        attributes(42);
        effect(42, Effect.JUMP_BOOST, 1, 100);
        assertEquals(0.3, movementSync.getMovementParameters().movementSpeed(), 1e-12);
        assertEquals((double) 0.2F, movementSync.getMovementParameters().jumpBoostVelocity());
        effect(42, Effect.JUMP_BOOST, 0, 0);
        assertEquals((double) 0.1F, movementSync.getMovementParameters().jumpBoostVelocity());
        listener.packetReceived(null, new ClientboundRemoveMobEffectPacket(41, Effect.JUMP_BOOST));
        assertEquals((double) 0.1F, movementSync.getMovementParameters().jumpBoostVelocity());
        listener.packetReceived(null, new ClientboundRemoveMobEffectPacket(42, Effect.JUMP_BOOST));
        assertEquals(0, movementSync.getMovementParameters().jumpBoostVelocity());
        effect(42, Effect.LEVITATION, 2, -1);
        assertEquals(2, movementSync.getMovementParameters().levitationAmplifier());
    }

    @Test
    void hungerAndBlindnessGateSprintAndFlightAbilityBypassesOnlyHunger() {
        listener.packetReceived(null, new ClientboundSetHealthPacket(20, 6, 0));
        assertFalse(movementSync.getMovementParameters().canSprint());
        assertEquals(movementSync.getMovementParameters().movementSpeed(), movementSync.getMovementParameters().movementSpeed(true));
        listener.packetReceived(null, new ClientboundSetHealthPacket(20, 7, 0));
        assertTrue(movementSync.getMovementParameters().canSprint());
        listener.packetReceived(null, new ClientboundSetHealthPacket(20, 0, 0));
        listener.packetReceived(null, new ClientboundPlayerAbilitiesPacket(false, true, false, false, 0.05F, 0.5F));
        assertTrue(movementSync.getMovementParameters().canSprint());
        assertEquals((double) 0.1F, movementSync.getMovementParameters().movementSpeed());
        effect(42, Effect.BLINDNESS, 0, 100);
        assertFalse(movementSync.getMovementParameters().canSprint());
    }

    @Test
    void respawnKeepsBasesAndOptionallyModifiersButNeverEffects() {
        attributes(42);
        effect(42, Effect.SLOW_FALLING, 0, 100);
        listener.packetReceived(null, new ClientboundRespawnPacket(spawn(), true, true));
        assertEquals(0.3, movementSync.getMovementParameters().movementSpeed(), 1e-12);
        assertFalse(movementSync.getMovementParameters().slowFalling());
        effect(42, Effect.LEVITATION, 0, -1);
        listener.packetReceived(null, new ClientboundRespawnPacket(spawn(), false, false));
        assertEquals(0.2, movementSync.getMovementParameters().movementSpeed());
        assertEquals(1.5, movementSync.getMovementParameters().stepHeight());
        assertEquals(-1, movementSync.getMovementParameters().levitationAmplifier());
    }

    @Test
    void loginDisconnectAndServerChangeClearState() {
        attributes(42);
        effect(42, Effect.BLINDNESS, 0, -1);
        listener.packetReceived(null, new ClientboundLoginPacket(42, false,
                new Key[]{Key.key("minecraft:overworld")}, 20, 12, 10, false, true, false, spawn(), false));
        assertEquals(MovementParameters.DEFAULT, movementSync.getMovementParameters());
        attributes(42);
        listener.disconnected(null);
        assertEquals(MovementParameters.DEFAULT, movementSync.getMovementParameters());
        attributes(42);
        new ServerChangeListener().onServerChange(null);
        assertEquals(MovementParameters.DEFAULT, movementSync.getMovementParameters());
    }
}
