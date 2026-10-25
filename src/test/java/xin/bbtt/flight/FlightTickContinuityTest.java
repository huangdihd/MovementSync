package xin.bbtt.flight;

import java.lang.reflect.Proxy;
import java.util.List;
import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PositionElement;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundPlayerPositionPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundAcceptTeleportationPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.listeners.TeleportPacketListener;
import xin.bbtt.physics.PlayerInput;

import static org.junit.jupiter.api.Assertions.*;

/** Only committed physics frames in one uninterrupted lifetime can prove flight progress. */
class FlightTickContinuityTest extends ElytraCommandTestSupport {
    @Test
    void committedFramePublishesMatchingClockHistoryAndDiagnosticVersions() {
        movementSync.setPhysicsTraceEnabled(true);
        long initialTick = movementSync.physicsTickVersion();
        long continuity = movementSync.flightTickContinuity();
        for (int i = 1; i <= 3; i++) {
            var trace = trace(i);
            movementSync.completePhysicsStep(trace);
            synchronized (movementSync) {
                long tick = initialTick + i;
                assertEquals(tick, movementSync.physicsTickVersion());
                assertEquals(continuity, movementSync.flightTickContinuity());
                assertEquals(trace, movementSync.getLastFlightTick());
                var samples = movementSync.flightTicksAfter(tick - 1, continuity);
                assertEquals(List.of(new MovementSync.FlightTickSample(tick, continuity, trace)), samples);
                var diagnostic = movementSync.getLastPhysicsTickTrace();
                assertEquals(tick, diagnostic.inputVersion());
                assertEquals(i, diagnostic.sequence());
                assertEquals(trace.position().x, diagnostic.x());
                assertEquals(trace.velocity().y, diagnostic.vy());
                assertTrue(movementSync.flightTicksAfter(tick, continuity).isEmpty(),
                        "the after cursor is exclusive");
            }
        }
    }

    @Test
    void diagnosticOnlyRecordingCannotCreateOrReplaceProofHistory() {
        movementSync.setPhysicsTraceEnabled(true);
        long tick = movementSync.physicsTickVersion();
        long continuity = movementSync.flightTickContinuity();
        movementSync.recordFlightTick(trace(100));
        assertEquals(tick, movementSync.physicsTickVersion());
        assertTrue(movementSync.flightTicksAfter(-1, continuity).isEmpty());
        assertEquals(tick, movementSync.getLastPhysicsTickTrace().inputVersion());

        var committed = trace(1);
        movementSync.completePhysicsStep(committed);
        var proof = movementSync.flightTicksAfter(-1, continuity);
        var diagnosticOnly = trace(200);
        movementSync.recordFlightTick(diagnosticOnly);
        movementSync.recordFlightTick(diagnosticOnly);
        assertEquals(tick + 1, movementSync.physicsTickVersion());
        assertEquals(diagnosticOnly, movementSync.getLastFlightTick());
        assertEquals(4, movementSync.getLastPhysicsTickTrace().sequence());
        assertEquals(tick + 1, movementSync.getLastPhysicsTickTrace().inputVersion());
        assertEquals(proof, movementSync.flightTicksAfter(-1, continuity));
        assertEquals(committed, proof.get(0).trace());
    }

    @Test
    void clockOnlyStepDoesNotInventAProofSample() {
        long continuity = movementSync.flightTickContinuity();
        long tick = movementSync.physicsTickVersion();
        movementSync.completePhysicsStep();
        assertEquals(tick + 1, movementSync.physicsTickVersion());
        assertTrue(movementSync.flightTicksAfter(-1, continuity).isEmpty());
        movementSync.completePhysicsStep(trace(2));
        assertEquals(tick + 2, movementSync.flightTicksAfter(-1, continuity).get(0).tick());
    }

    @Test
    void historyRetainsOnlyNewest64FramesInTickOrder() {
        long tick = movementSync.physicsTickVersion();
        long continuity = movementSync.flightTickContinuity();
        for (int i = 1; i <= 70; i++) movementSync.completePhysicsStep(trace(i));
        var samples = movementSync.flightTicksAfter(-1, continuity);
        assertEquals(64, samples.size());
        for (int i = 0; i < samples.size(); i++) {
            var sample = samples.get(i);
            assertEquals(tick + i + 7, sample.tick());
            assertEquals(continuity, sample.continuity());
            assertEquals(trace(i + 7), sample.trace());
        }
        assertEquals(samples.subList(63, 64), movementSync.flightTicksAfter(tick + 69, continuity));
        assertTrue(movementSync.flightTicksAfter(-1, continuity + 1).isEmpty());
        assertThrows(UnsupportedOperationException.class, samples::clear);
        movementSync.completePhysicsStep(trace(71));
        assertEquals(tick + 7, samples.get(0).tick(), "returned history is a detached snapshot");
        assertEquals(tick + 8, movementSync.flightTicksAfter(-1, continuity).get(0).tick());
    }

    @Test
    void traceCopiesInputVectorsAndDoesNotExposeMutableVectors() {
        var position = new Vector3d(1, 80, 3);
        var velocity = new Vector3d(.5, -.2, .3);
        var trace = new MovementSync.FlightTickTrace(position, velocity, false, false, false,
                true, false, 15f, -5f, PlayerInput.NONE, 2.0);
        position.set(999, 999, 999);
        velocity.zero();
        movementSync.completePhysicsStep(trace);
        var sample = movementSync.flightTicksAfter(-1, movementSync.flightTickContinuity()).get(0);
        trace.position().zero();
        trace.velocity().zero();
        sample.trace().position().zero();
        sample.trace().velocity().zero();
        movementSync.getLastFlightTick().position().zero();
        movementSync.getLastFlightTick().velocity().zero();
        assertEquals(new Vector3d(1, 80, 3), sample.trace().position());
        assertEquals(new Vector3d(.5, -.2, .3), sample.trace().velocity());
        assertEquals(new Vector3d(1, 80, 3), movementSync.getLastFlightTick().position());
        assertEquals(new Vector3d(.5, -.2, .3), movementSync.getLastFlightTick().velocity());
    }

    @Test
    void explicitInvalidationClearsHistoryAndAdvancesContinuity() {
        assertBoundary(movementSync::invalidateFlightTickContinuity);
    }

    @Test
    void newSessionClearsHistoryAndAdvancesContinuity() {
        assertBoundary(movementSync::resetFlightMetadataSession);
    }

    @Test
    void respawnClearsHistoryAndAdvancesContinuity() {
        assertBoundary(movementSync::onPlayerRespawn);
    }

    @Test
    void deathAndSubsequentRespawnEachStartNewContinuity() {
        assertBoundary(movementSync::onPlayerDeath);
        assertTrue(movementSync.isDead());
        assertBoundary(movementSync::onPlayerRespawn);
        assertFalse(movementSync.isDead());
    }

    @Test
    void dimensionChangeClearsHistoryButSameDimensionDoesNot() {
        movementSync.setRecoveryDimension("minecraft:overworld");
        movementSync.completePhysicsStep(trace(1));
        long continuity = movementSync.flightTickContinuity();
        var samples = movementSync.flightTicksAfter(-1, continuity);
        movementSync.setRecoveryDimension("minecraft:overworld");
        assertEquals(continuity, movementSync.flightTickContinuity());
        assertEquals(samples, movementSync.flightTicksAfter(-1, continuity));
        assertBoundary(() -> movementSync.setRecoveryDimension("minecraft:the_nether"));
        assertBoundary(() -> movementSync.setRecoveryDimension(null));
    }

    @Test
    void smallRelativeCorrectionBreaksProofContinuityWithoutRetiringControlEpoch() {
        assertTeleportBoundary(.029331, false);
    }

    @Test
    void farRelativeTeleportBreaksProofContinuityAndRetiresControlEpoch() {
        assertTeleportBoundary(230, true);
    }

    private void assertBoundary(Runnable boundary) {
        long continuity = movementSync.flightTickContinuity();
        movementSync.completePhysicsStep(trace(1));
        long tick = movementSync.physicsTickVersion();
        assertFalse(movementSync.flightTicksAfter(-1, continuity).isEmpty());
        boundary.run();
        assertEquals(continuity + 1, movementSync.flightTickContinuity());
        assertEquals(tick, movementSync.physicsTickVersion(), "a boundary is not a physics step");
        assertTrue(movementSync.flightTicksAfter(-1, continuity).isEmpty());
        assertTrue(movementSync.flightTicksAfter(-1, continuity + 1).isEmpty());
        var next = trace(2);
        movementSync.completePhysicsStep(next);
        assertTrue(movementSync.flightTicksAfter(-1, continuity).isEmpty(),
                "an old continuity token cannot see new frames");
        assertEquals(List.of(new MovementSync.FlightTickSample(tick + 1, continuity + 1, next)),
                movementSync.flightTicksAfter(tick, continuity + 1));
    }

    private void assertTeleportBoundary(double distance, boolean retiresControlEpoch) {
        movementSync.position.set(new Vector3d(10, 80, -5));
        movementSync.velocity.set(new Vector3d(1, 0, 0));
        movementSync.onRecoveryTeleport();
        movementSync.setFollowTargetId(99);
        movementSync.setGliding(true);
        movementSync.claimFlightControlOwner(this, xin.bbtt.physics.FlightControlAdmission.Mode.LOCAL_SOLVER);
        movementSync.completePhysicsStep(trace(1));
        long continuity = movementSync.flightTickContinuity();
        long tick = movementSync.physicsTickVersion();
        long epoch = movementSync.flightControlEpoch();
        Session observingSession = (Session) Proxy.newProxyInstance(Session.class.getClassLoader(),
                new Class<?>[]{Session.class}, (proxy, method, arguments) -> {
                    if (method.getName().equals("send")) {
                        assertEquals(continuity + 1, movementSync.flightTickContinuity(),
                                "proof continuity must retire before teleport acknowledgement");
                        assertTrue(movementSync.flightTicksAfter(-1, continuity).isEmpty());
                        assertTrue(movementSync.flightTicksAfter(-1, continuity + 1).isEmpty());
                        packets.add(arguments[0]);
                    }
                    return null;
                });
        new TeleportPacketListener().packetReceived(observingSession,
                new ClientboundPlayerPositionPacket(17, distance, 0, 0, 0, 0, 0, 0, 0,
                        PositionElement.X, PositionElement.Y, PositionElement.Z));
        assertEquals(17, assertInstanceOf(ServerboundAcceptTeleportationPacket.class, packets.get(0)).getId());
        assertInstanceOf(ServerboundMovePlayerPosRotPacket.class, packets.get(1));
        assertEquals(new Vector3d(10 + distance, 80, -5), movementSync.position.get());
        assertEquals(continuity + 1, movementSync.flightTickContinuity());
        assertEquals(epoch + (retiresControlEpoch ? 1 : 0), movementSync.flightControlEpoch());
        assertEquals(tick, movementSync.physicsTickVersion());
        movementSync.completePhysicsStep(trace(2));
        assertTrue(movementSync.flightTicksAfter(-1, continuity).isEmpty());
        assertEquals(1, movementSync.flightTicksAfter(tick, continuity + 1).size());
    }

    private static MovementSync.FlightTickTrace trace(int frame) {
        return new MovementSync.FlightTickTrace(new Vector3d(frame, 80, 0),
                new Vector3d(1, -.1, 0), false, false, false, true, false,
                0f, 0f, PlayerInput.NONE, 0.0);
    }
}
