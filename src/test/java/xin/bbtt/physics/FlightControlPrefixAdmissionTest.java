package xin.bbtt.physics;

import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class FlightControlPrefixAdmissionTest extends ElytraCommandTestSupport {
    private final FlightControlAdmission admission = new FlightControlAdmission();
    private final Object owner = new Object();

    private void setupFlight() throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        });
        movementSync.position.set(new Vector3d(0, 100, 0));
        movementSync.velocity.set(new Vector3d(0, 0, .3));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        admission.claim(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
    }

    private FlightControlAdmission.Source source() {
        var snapshot = FlightControlAdmission.snapshot(movementSync);
        return new FlightControlAdmission.Source(snapshot.tick(), snapshot.generation(), snapshot.state(), admission.epoch());
    }

    private List<FlightControlAdmission.Step> steps(float yaw, float pitch) {
        return List.of(new FlightControlAdmission.Step(new ElytraFlightController.Decision(yaw, pitch,
                false, ElytraFlightController.Phase.CRUISE), source().state()));
    }

    @Test void slowWorkerExhaustsExactPrefixThenFixedRecoveryThenForwardEmergency() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var prefix = steps(0, 0);
            assertTrue(FlightControlAdmission.createPrefix(owner, source(), prefix, steps(180, 25),
                    null, () -> {}).recovery().isEmpty(), "reverse descent is never a braking reserve");
            var recovery = steps(0, -45);
            var envelope = FlightControlAdmission.createPrefix(owner, source(), prefix, recovery, null, () -> {});
            assertEquals(prefix, envelope.steps(), "no appended terminal rollout");
            assertEquals(recovery, envelope.recovery());
            assertTrue(admission.registerForwardEmergency(movementSync, owner, source(), 0));
            admission.submit(envelope);
            admission.consume(movementSync);
            assertTrue(admission.ownershipTrace().active());
            movementSync.completePhysicsStep();
            admission.consume(movementSync);
            assertTrue(admission.ownershipTrace().recovering());
            movementSync.completePhysicsStep();
            admission.consume(movementSync);
            assertFalse(admission.ownershipTrace().active());
            assertEquals(0f, movementSync.yaw.get() % 360f, .001f);
            assertEquals(-20f, movementSync.pitch.get());
            assertTrue(admission.failure().reason().startsWith("UNPROVEN"));
            assertNull(movementSync.pollBeforeMovementPacket());
            for (int i = 0; i < 12; i++) {
                movementSync.completePhysicsStep();
                admission.consume(movementSync);
                assertEquals(-20f, movementSync.pitch.get());
                assertEquals(0f, movementSync.yaw.get() % 360f, .001f);
            }
        }
    }

    @Test void staleEpochAndGenerationCannotPublishOrReplaceForwardHeading() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            var current = source();
            assertTrue(admission.registerForwardEmergency(movementSync, owner, current, 0));
            for (var stale : List.of(
                    new FlightControlAdmission.Source(current.tick(), current.generation(), current.state(), current.epoch() - 1),
                    new FlightControlAdmission.Source(current.tick(), current.generation() - 1, current.state(), current.epoch()))) {
                assertFalse(admission.registerForwardEmergency(movementSync, owner, stale, 180));
                assertFalse(admission.publishPrefix(movementSync, owner, stale, steps(180, 25), List.of(), null, () -> fail("stale callback")));
            }
            admission.consume(movementSync);
            assertEquals(-20f, movementSync.pitch.get());
            assertEquals(0f, movementSync.yaw.get() % 360f, .001f);
        }
    }

    @Test void invalidPendingPreservesValidActivePrefix() throws Exception {
        setupFlight();
        synchronized (movementSync) {
            assertTrue(admission.publishPrefix(movementSync, owner, source(), steps(0, -5), List.of(), null, () -> {}));
            admission.consume(movementSync);
            long activation = admission.ownershipTrace().activation();
            assertTrue(admission.publishPrefix(movementSync, owner, source(), List.of(), List.of(), null, () -> fail("empty prefix")));
            admission.consume(movementSync);
            assertEquals(activation, admission.ownershipTrace().activation());
            assertTrue(admission.ownershipTrace().active());
            assertEquals(-5f, movementSync.pitch.get());
        }
    }
}
