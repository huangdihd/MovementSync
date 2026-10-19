package xin.bbtt.listeners;

import java.lang.reflect.Proxy;
import java.util.Map;
import net.kyori.adventure.key.Key;
import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.GameMode;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerSpawnInfo;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PositionElement;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundRespawnPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundPlayerPositionPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundSetHealthPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Synthetic loaded ground; regression enters through the actual authoritative packet listener. */
class AuthoritativeLandingMemoryLifecycleTest {
    private final MovementSync previous = MovementSync.INSTANCE;
    private final MovementSync sync = new MovementSync();
    private final World ground = new World() {
        private final BlockState stone = new BlockState("minecraft:netherrack", 1, Map.of(), "block", 1.5, true, "STONE");
        private final BlockState air = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 34 ? stone : air; }
    };
    private final Session session = (Session) Proxy.newProxyInstance(getClass().getClassLoader(),
            new Class<?>[]{Session.class}, (proxy, method, args) -> null);

    @BeforeEach void seed() {
        MovementSync.INSTANCE = sync;
        sync.setRecoveryDimension("minecraft:the_nether");
        sync.position.set(new Vector3d(227.5, 34, 21.5));
        sync.velocity.set(new Vector3d(1.5, -.1, 0));
        sync.onRecoveryTeleport();
        sync.getLandingMemory().takeoff(ground, sync.position.get());
        sync.getLandingMemory().landed(ground, sync.position.get());
        sync.getLandingMemory().observe(ground, sync.position.get());
        assertFalse(sync.getLandingMemory().snapshot().isEmpty());
    }

    @AfterEach void restore() { MovementSync.INSTANCE = previous; }

    @Test void farAbsoluteCorrectionForgetsOldDepartureTouchdownAndObservations() {
        correct(new ClientboundPlayerPositionPacket(1, -3.5, 36, -1.5, 0, 0, 0, 0, 0));
        assertEquals(new Vector3d(-3.5, 36, -1.5), sync.position.get());
        assertTrue(sync.getLandingMemory().snapshot().isEmpty(), "A ~230 blocks away must not remain a B return anchor");
        assertNull(sync.getLandingMemory().nearest(sync.position.get()));
        sync.getLandingMemory().takeoff(ground, new Vector3d(-3.5, 34, -1.5));
        assertEquals(new Vector3d(-3.5, 34, -1.5), sync.getLandingMemory().nearest(sync.position.get()),
                "the new departure must be pinnable after correction");
    }

    @Test void farRelativeCorrectionUsesResolvedTargetAndPreservesFollowFlight() {
        sync.setFollowTargetId(99);
        sync.setGliding(true);
        long generation = sync.getNavigationGeneration();
        correct(new ClientboundPlayerPositionPacket(2, -231, 2, -23, 0, 0, 0, 0, 0,
                PositionElement.X, PositionElement.Y, PositionElement.Z));
        assertTrue(sync.getLandingMemory().snapshot().isEmpty());
        assertTrue(sync.isGliding());
        assertEquals(99, sync.getFollowTargetId());
        assertEquals(generation, sync.getNavigationGeneration());
    }

    @Test void smallGrimCorrectionRetainsSitesButStillInvalidatesNavigation() {
        var sites = sync.getLandingMemory().snapshot();
        long generation = sync.getNavigationGeneration();
        correct(new ClientboundPlayerPositionPacket(3, .029331, 0, 0, 0, 0, 0, 0, 0,
                PositionElement.X, PositionElement.Y, PositionElement.Z));
        assertEquals(sites, sync.getLandingMemory().snapshot());
        assertTrue(sync.getNavigationGeneration() > generation);
    }

    @Test void plausibleFastOneTickCorrectionRetainsSites() {
        sync.velocity.set(new Vector3d(12, 0, 0));
        var sites = sync.getLandingMemory().snapshot();
        correct(new ClientboundPlayerPositionPacket(4, 12, 0, 0, 0, 0, 0, 0, 0,
                PositionElement.X, PositionElement.Y, PositionElement.Z));
        assertEquals(sites, sync.getLandingMemory().snapshot());
    }

    @Test void verticalDiscontinuityAlsoClearsSites() {
        correct(new ClientboundPlayerPositionPacket(5, 0, 230, 0, 0, 0, 0, 0, 0,
                PositionElement.X, PositionElement.Y, PositionElement.Z));
        assertTrue(sync.getLandingMemory().snapshot().isEmpty());
    }

    @Test void dimensionChangeClearsSites() {
        sync.setRecoveryDimension("minecraft:overworld");
        assertTrue(sync.getLandingMemory().snapshot().isEmpty());
    }

    @Test void deathPacketClearsSites() {
        new MovementParametersPacketListener().packetReceived(null, new ClientboundSetHealthPacket(0, 20, 5));
        assertTrue(sync.getLandingMemory().snapshot().isEmpty());
    }

    @Test void sameDimensionRespawnPacketClearsSitesWithoutPriorDeathPacket() {
        var spawn = new PlayerSpawnInfo(0, Key.key("minecraft:the_nether"), 0, GameMode.SURVIVAL,
                null, false, true, null, 0, 63);
        new LoginPacketListener().packetReceived(null, new ClientboundRespawnPacket(spawn, false, false));
        assertTrue(sync.getLandingMemory().snapshot().isEmpty());
    }

    private void correct(ClientboundPlayerPositionPacket packet) {
        new TeleportPacketListener().packetReceived(session, packet);
    }
}
