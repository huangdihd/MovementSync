package xin.bbtt.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** P1 boundary regressions: synthetic finite leases, with a real physical clock. */
class FlightControlP1Test extends ElytraCommandTestSupport {
    private static final class ResponsibleOwner {}
    private final Object owner = new ResponsibleOwner();

    @BeforeEach void loadedEmptyWorld() throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        });
        movementSync.position.set(new Vector3d(0, 100, 0));
        movementSync.velocity.set(new Vector3d(0, 0, .3));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL);
    }

    private FlightControlAdmission.Envelope envelope(FlightControlAdmission.Source source,
            AtomicInteger accepted, boolean firework, float... pitches) {
        var steps = new ArrayList<FlightControlAdmission.Step>();
        var state = source.state();
        for (float pitch : pitches) {
            var decision = new ElytraFlightController.Decision(0, pitch,
                    steps.isEmpty() && firework, ElytraFlightController.Phase.CRUISE);
            steps.add(new FlightControlAdmission.Step(decision, state));
            var result = new GlidingPhysics(0, pitch, false, PlayerInput.NONE,
                    movementSync.getMovementParameters()).step(movementSync.getWorld(), state,
                    PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            state = FlightSafety.nextState(result);
        }
        return new FlightControlAdmission.Envelope(owner, source, steps, null,
                accepted::incrementAndGet, List.of());
    }

    private void physicalStep() {
        var input = movementSync.snapshotInput();
        var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                movementSync.onGround.get(), movementSync.fallDistance.get(), PlayerPose.GLIDING);
        var result = new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(), false,
                input, movementSync.getMovementParameters()).step(movementSync.getWorld(), state,
                PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        assertFalse(result.collidedX());
        assertFalse(result.collidedZ());
        movementSync.position.set(result.position());
        movementSync.velocity.set(result.velocity());
        movementSync.onGround.set(result.onGround());
        movementSync.fallDistance.set(result.fallDistance());
        movementSync.completePhysicsStep();
    }

    @Test void unsentBatchActionIsDiscardedAfterModeHandoff() {
        var source = FlightControlAdmission.snapshot(movementSync);
        var prepared = envelope(source, new AtomicInteger(), false, 0);
        var packet = new org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket(
                org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand.MAIN_HAND, 0, 0f, 0f);
        var withAction = new FlightControlAdmission.Envelope(owner, source, prepared.steps(), null,
                () -> movementSync.queueBeforeMovement(packet), List.of());
        movementSync.submitFlightControls(withAction);
        movementSync.snapshotInput();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.DIRECT);
        assertNull(movementSync.pollBeforeMovementPacket());
    }

    @Test void cancellationBetweenSnapshotAndLateSubmitRejectsControlsAndCallback() {
        var source = FlightControlAdmission.snapshot(movementSync);
        movementSync.revokeFlightControls(owner);
        var accepted = new AtomicInteger();
        var late = envelope(source, accepted, true, 25);
        movementSync.pitch.set(-7f);
        movementSync.submitFlightControls(late);
        physicalStep();
        assertEquals(0, accepted.get());
        assertEquals(-7f, movementSync.pitch.get());
        assertNull(movementSync.pollBeforeMovementPacket());
    }

    @Test void modeSwitchInvalidatesActiveBatchAndLateOldSubmission() {
        var accepted = new AtomicInteger();
        var source = FlightControlAdmission.snapshot(movementSync);
        var active = envelope(source, accepted, false, 10, 25);
        var late = envelope(source, accepted, true, 30);
        movementSync.submitFlightControls(active);
        physicalStep();
        assertEquals(1, accepted.get());
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.DIRECT);
        movementSync.pitch.set(-7f);
        movementSync.submitFlightControls(late);
        physicalStep();
        assertEquals(-7f, movementSync.pitch.get());
        assertEquals(1, accepted.get());
        assertNull(movementSync.getFlightControlFailure());
    }

    @Test void duplicateFireworkEnvelopeAcceptsCallbackExactlyOnce() {
        var accepted = new AtomicInteger();
        var envelope = envelope(FlightControlAdmission.snapshot(movementSync), accepted, true, 10, 20);
        movementSync.submitFlightControls(envelope);
        movementSync.submitFlightControls(envelope);
        movementSync.snapshotInput();
        assertEquals(1, accepted.get());
        movementSync.submitFlightControls(envelope);
        physicalStep();
        movementSync.submitFlightControls(envelope);
        physicalStep();
        assertEquals(1, accepted.get());
        assertEquals(20f, movementSync.pitch.get(), "resubmission must not restart the lease");
    }

    @Test void inputSnapshotsDoNotAdvanceCommandDeadlineOrSourceAge() {
        var accepted = new AtomicInteger();
        var source = FlightControlAdmission.snapshot(movementSync);
        var envelope = envelope(source, accepted, false, 10, 20);
        for (int i = 0; i < 20; i++) movementSync.snapshotInput();
        assertEquals(source.tick(), movementSync.physicsTickVersion());
        movementSync.submitFlightControls(envelope);
        for (int i = 0; i < 20; i++) {
            movementSync.snapshotInput();
            assertEquals(10f, movementSync.pitch.get());
            assertNull(movementSync.getFlightControlFailure());
        }
        assertEquals(1, accepted.get());
        assertEquals(source.tick(), movementSync.physicsTickVersion());
        physicalStep();
        movementSync.snapshotInput();
        assertEquals(source.tick() + 1, movementSync.physicsTickVersion());
        assertEquals(20f, movementSync.pitch.get());
    }

    @Test void exhaustedContinuationReportsResponsibleOwnerWithoutFabricatingDive() {
        var accepted = new AtomicInteger();
        var source = FlightControlAdmission.snapshot(movementSync);
        movementSync.submitFlightControls(envelope(source, accepted, false, 10, 20));
        physicalStep();
        assertNull(movementSync.getFlightControlFailure());
        physicalStep();
        movementSync.snapshotInput();
        var failure = movementSync.getFlightControlFailure();
        assertNotNull(failure);
        assertEquals(source.tick() + 2, failure.tick());
        assertEquals(source.generation(), failure.generation());
        assertEquals(source.epoch(), failure.epoch());
        assertEquals(ResponsibleOwner.class.getName(), failure.responsibleOwner());
        assertTrue(failure.reason().contains("UNPROVEN_EMERGENCY"));
        assertTrue(failure.reason().contains("exhausted"));
        assertEquals(20f, movementSync.pitch.get(), "exhaustion must not manufacture pitch 90");
        assertTrue(movementSync.isGliding());
        assertFalse(movementSync.onGround.get());
        assertEquals(1, accepted.get());
    }

    @Test void tickBatchAuthorizesOnlyItsExactPhysicalTick() {
        var envelope = envelope(FlightControlAdmission.snapshot(movementSync), new AtomicInteger(), true, 10, 20);
        var batch = TickControlBatch.admitted(envelope, 17);
        assertEquals(envelope.source().generation(), batch.generation());
        assertEquals(envelope.source().epoch(), batch.epoch());
        assertTrue(batch.authorizes(0, 17));
        assertFalse(batch.authorizes(0, 18));
        assertFalse(batch.authorizes(1, 17));
        assertTrue(batch.authorizes(1, 18));
        assertFalse(batch.authorizes(2, 19));
        assertFalse(batch.authorizes(-1, 16));
        assertTrue(batch.commands().get(0).firework());
        assertFalse(batch.commands().get(1).firework());
    }
}
