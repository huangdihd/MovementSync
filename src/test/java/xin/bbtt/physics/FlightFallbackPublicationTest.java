package xin.bbtt.physics;

import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class FlightFallbackPublicationTest extends ElytraCommandTestSupport {
    private final Object owner = new Object();

    @BeforeEach
    void setupFlight() throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 0 ? new BlockState("minecraft:stone", 1, Map.of(),
                        "block", 1.5, true, "STONE") : super.getBlockStateAt(position);
            }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double maximumX, double maximumY, double maximumZ) {
                return y < 0 || maximumX > 3;
            }
        });
        movementSync.position.set(new Vector3d(0, 20, 0));
        movementSync.velocity.set(new Vector3d(0, 0, .3));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
    }

    private FlightControlAdmission.Envelope envelope(Object candidateOwner, boolean rocket,
            AtomicInteger accepted) {
        var source = FlightControlAdmission.snapshot(movementSync);
        var control = new ElytraFlightController.Decision(0, 0, rocket,
                ElytraFlightController.Phase.CRUISE);
        var route = new WaypointFlightTarget(List.of(new Vector3d(0, 20, 0), new Vector3d(0, 20, 100)),
                new Vector3d(0, 20, 100), false, false, 3);
        var envelope = FlightControlAdmission.prepare(movementSync, candidateOwner, source,
                control, route, accepted::incrementAndGet);
        assertFalse(envelope.steps().isEmpty(), "fixture must prepare executable controls");
        return envelope;
    }

    private Object admissionField(String name) throws Exception {
        var admission = MovementSync.class.getDeclaredField("flightControls");
        admission.setAccessible(true);
        var field = FlightControlAdmission.class.getDeclaredField(name);
        field.setAccessible(true);
        return field.get(admission.get(movementSync));
    }

    @Test void fallbackPreservesPendingRocketIdentityAndAcceptsItExactlyOnce() throws Exception {
        var rocketAccepted = new AtomicInteger();
        var fallbackAccepted = new AtomicInteger();
        var rocket = envelope(owner, true, rocketAccepted);
        var fallback = envelope(owner, false, fallbackAccepted);
        assertTrue(rocket.steps().get(0).control().firework());

        movementSync.submitFlightControls(rocket);
        movementSync.submitFallbackFlightControls(fallback);
        assertSame(rocket, admissionField("pending"));
        assertEquals(0, rocketAccepted.get(), "publication is not admission");
        movementSync.snapshotInput();
        assertSame(rocket, admissionField("active"));
        movementSync.snapshotInput();
        movementSync.submitFallbackFlightControls(rocket);
        assertNull(admissionField("pending"), "consumed identities cannot be submitted again");
        movementSync.snapshotInput();
        assertEquals(1, rocketAccepted.get());
        assertEquals(0, fallbackAccepted.get());
    }

    @Test void fallbackPublishesIntoEmptyPendingSlot() throws Exception {
        var accepted = new AtomicInteger();
        var fallback = envelope(owner, false, accepted);
        movementSync.submitFallbackFlightControls(fallback);
        assertSame(fallback, admissionField("pending"));
        assertEquals(0, accepted.get());
        movementSync.snapshotInput();
        assertSame(fallback, admissionField("active"));
        assertEquals(1, accepted.get());
    }

    @Test void normalPublicationStillReplacesPendingFallbackAndNormalPublication() throws Exception {
        var discarded = new AtomicInteger();
        var accepted = new AtomicInteger();
        movementSync.submitFallbackFlightControls(envelope(owner, false, discarded));
        movementSync.submitFlightControls(envelope(owner, false, discarded));
        var latest = envelope(owner, true, accepted);
        movementSync.submitFlightControls(latest);
        assertSame(latest, admissionField("pending"));
        movementSync.snapshotInput();
        assertSame(latest, admissionField("active"));
        assertEquals(1, accepted.get());
        assertEquals(0, discarded.get());
    }

    @Test void fallbackCannotShortenTheActiveExecutableHorizon() throws Exception {
        var source = FlightControlAdmission.snapshot(movementSync);
        var control = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);
        var step = new FlightControlAdmission.Step(control, source.state());
        var reserve = java.util.Collections.nCopies(6, step);
        var initial = FlightControlAdmission.createPrefix(owner, source,
                java.util.Collections.nCopies(10, step), reserve, null, () -> {});
        movementSync.submitFlightControls(initial);
        movementSync.snapshotInput();
        assertSame(initial, admissionField("active"));
        movementSync.completePhysicsStep();
        var accepted = new AtomicInteger();
        var fallback = FlightControlAdmission.createPrefix(owner,
                FlightControlAdmission.snapshot(movementSync), java.util.Collections.nCopies(3, step),
                reserve, null, accepted::incrementAndGet);
        movementSync.submitFallbackFlightControls(fallback);
        assertSame(fallback, admissionField("pending"));
        movementSync.snapshotInput();
        assertSame(initial, admissionField("active"));
        assertEquals(0, accepted.get());
    }

    @Test void foreignFallbackIsRejectedAtSubmission() throws Exception {
        var accepted = new AtomicInteger();
        movementSync.submitFallbackFlightControls(envelope(new Object(), true, accepted));
        assertNull(admissionField("pending"));
        movementSync.snapshotInput();
        assertEquals(0, accepted.get());
    }

    @Test void revokedFallbackCannotRepublishAnOldEpoch() throws Exception {
        var accepted = new AtomicInteger();
        var fallback = envelope(owner, true, accepted);
        movementSync.submitFallbackFlightControls(fallback);
        movementSync.revokeFlightControls(owner);
        movementSync.submitFallbackFlightControls(fallback);
        assertNull(admissionField("pending"));
        movementSync.snapshotInput();
        assertEquals(0, accepted.get());
    }

    @Test void expiredFallbackStillFailsAtConsumption() throws Exception {
        var accepted = new AtomicInteger();
        var fallback = envelope(owner, true, accepted);
        for (int tick = 0; tick < 9; tick++) movementSync.completePhysicsStep();
        movementSync.submitFallbackFlightControls(fallback);
        assertSame(fallback, admissionField("pending"), "age remains a consumption-time gate");
        movementSync.snapshotInput();
        assertEquals(0, accepted.get());
        assertNull(admissionField("pending"));
    }

    @Test void fallbackDoesNotBypassPhysicalEnvelopeRevalidation() throws Exception {
        var accepted = new AtomicInteger();
        var fallback = envelope(owner, true, accepted);
        movementSync.submitFallbackFlightControls(fallback);
        movementSync.position.set(new Vector3d(2.69, 20, 0));
        movementSync.velocity.set(new Vector3d(.4, 0, 0));
        movementSync.snapshotInput();
        assertEquals(0, accepted.get());
        assertNull(movementSync.pollBeforeMovementPacket());
    }

    @Test void wrongGenerationFallbackStillFailsAtConsumption() throws Exception {
        var accepted = new AtomicInteger();
        var prepared = envelope(owner, true, accepted);
        var source = prepared.source();
        var foreignSource = new FlightControlAdmission.Source(source.tick(), source.generation() + 1,
                source.state(), source.epoch());
        var fallback = new FlightControlAdmission.Envelope(owner, foreignSource, prepared.steps(),
                prepared.corridor(), prepared.accepted(), prepared.recovery());
        movementSync.submitFallbackFlightControls(fallback);
        movementSync.snapshotInput();
        assertEquals(0, accepted.get());
    }
}
