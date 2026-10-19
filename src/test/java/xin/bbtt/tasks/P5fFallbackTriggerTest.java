package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.*;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.physics.*;

/** Independent regression cases for hazard trigger and masked ceiling travel. */
class P5fFallbackTriggerTest extends P5eFallbackOwnerTest {
    @Test void fastUnownedSinkingNearLavaStillTriggersFallback() {
        lava = true;
        sync.velocity.set(new Vector3d(0, -.2, 1.2));
        motion.run();
        assertEquals(-55f, sync.pitch.get());
        assertEquals(1, uses());
    }

    @Test void slowUnownedRisingNearLavaStillTriggersFallback() {
        lava = true;
        sync.velocity.set(new Vector3d(0, .1, .2));
        motion.run();
        assertEquals(-55f, sync.pitch.get());
        assertEquals(1, uses());
    }

    @Test void fastRisingNearLavaRetainsExternalControl() {
        lava = true;
        sync.velocity.set(new Vector3d(0, .2, 1.2));
        sync.pitch.set(17f);
        motion.run();
        assertEquals(17f, sync.pitch.get());
        assertEquals(0, uses());
    }

    @Test void attachedAdmissionRejectsThrustMaskedCeilingContact() {
        ceiling = true;
        attach();
        sync.position.set(new Vector3d(.5, 12 - PlayerPose.GLIDING.height(), .5));
        sync.velocity.set(new Vector3d(0, .4, .3));
        var accepted = submit(false, 1);
        sync.pitch.set(17f);
        motion.run();
        assertFalse(accepted.get());
        assertEquals(17f, sync.pitch.get());
    }

    @Test void poweredAdmissionRejectsLaterCeilingContactBeforeRequest() {
        ceiling = true;
        sync.position.set(new Vector3d(.5, 10.5, .5));
        sync.velocity.set(new Vector3d(0, 0, .3));
        var accepted = submit(true, 6);
        motion.run();
        assertFalse(accepted.get(), "powered replay must reject contact from its own evolving state");
    }

    private AtomicBoolean submit(boolean rocket, int ticks) {
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.DIRECT);
        var source = FlightControlAdmission.snapshot(sync);
        var control = new ElytraFlightController.Decision(0, -20, rocket,
                ElytraFlightController.Phase.CRUISE);
        var accepted = new AtomicBoolean();
        var steps = java.util.stream.IntStream.range(0, ticks)
                .mapToObj(i -> new FlightControlAdmission.Step(control, source.state())).toList();
        sync.submitFlightControls(new FlightControlAdmission.Envelope(this, source, steps,
                null, () -> accepted.set(true), List.of()));
        return accepted;
    }
}
