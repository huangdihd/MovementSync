package xin.bbtt.listeners;

import java.lang.reflect.Proxy;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PositionElement;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundPlayerPositionPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.FlightControlAdmission;

import static org.junit.jupiter.api.Assertions.*;

/** Paused-worker ordering: capture at A, deliver packet B, then attempt old publication. */
class TeleportFlightEpochTest {
    private final MovementSync previous = MovementSync.INSTANCE;
    private final MovementSync sync = new MovementSync();
    private final Object owner = new Object();
    private final Session session = (Session) Proxy.newProxyInstance(getClass().getClassLoader(),
            new Class<?>[]{Session.class}, (proxy, method, args) -> null);

    @BeforeEach void setup() {
        MovementSync.INSTANCE = sync;
        sync.position.set(new Vector3d(227.5, 34, 21.5));
        sync.velocity.set(new Vector3d(1.5, -.1, 0));
        sync.onRecoveryTeleport();
        sync.setFollowTargetId(99);
        sync.setGliding(true);
        sync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
    }

    @AfterEach void restore() { MovementSync.INSTANCE = previous; }

    @Test void farAbsolutePacketRevokesOldWorkerPublicationBeforeAcknowledgement() {
        assertFar(new ClientboundPlayerPositionPacket(1, -3.5, 36, -1.5, 500, 0, 0, 0, 0));
    }

    @Test void farRelativePacketRevokesOldWorkerPublication() {
        assertFar(relative(-231, 2, -23));
    }

    @Test void verticalDiscontinuityRevokesOldWorkerPublication() {
        assertFar(relative(0, 230, 0));
    }

    private void assertFar(ClientboundPlayerPositionPacket packet) {
        var source = FlightControlAdmission.snapshot(sync);
        AtomicBoolean accepted = new AtomicBoolean();
        Session observingSession = (Session) Proxy.newProxyInstance(getClass().getClassLoader(),
                new Class<?>[]{Session.class}, (proxy, method, args) -> {
                    if (method.getName().equals("send"))
                        assertTrue(sync.flightControlEpoch() > source.epoch(),
                                "publication rights must retire before teleport acknowledgement");
                    return null;
                });
        new TeleportPacketListener().packetReceived(observingSession, packet);
        assertEquals(source.epoch() + 1, sync.flightControlEpoch());
        assertEquals(source.generation(), sync.getNavigationGeneration());
        assertEquals(99, sync.getFollowTargetId());
        assertTrue(sync.isGliding());
        // Reclaiming the same worker owner must not resurrect its old source token.
        sync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        assertFalse(sync.registerForwardFlightEmergency(owner, source, 90));
        assertFalse(sync.publishFlightPrefix(owner, source, List.of(), List.of(), null,
                () -> accepted.set(true)));
        sync.snapshotInput();
        assertFalse(accepted.get());
    }

    @Test void smallCorrectionPreservesCurrentWorkerToken() {
        assertNormal(.029331);
    }

    @Test void plausibleFastCorrectionPreservesCurrentWorkerToken() {
        sync.velocity.set(new Vector3d(12, 0, 0));
        assertNormal(12);
    }

    @Test void toleranceBoundaryPreservesCurrentWorkerToken() {
        assertNormal(8);
    }

    private void assertNormal(double distance) {
        var source = FlightControlAdmission.snapshot(sync);
        new TeleportPacketListener().packetReceived(session, relative(distance, 0, 0));
        assertEquals(source.epoch(), sync.flightControlEpoch());
        assertTrue(sync.registerForwardFlightEmergency(owner, source, 90));
    }

    private ClientboundPlayerPositionPacket relative(double x, double y, double z) {
        return new ClientboundPlayerPositionPacket(2, x, y, z, 0, 0, 0, 0, 0,
                PositionElement.X, PositionElement.Y, PositionElement.Z);
    }
}
