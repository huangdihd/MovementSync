package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.*;
import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.physics.*;

/** Review regressions through actual admission + updateMotionTask consumption. */
class P5fAdmissionReviewTest extends P5eFallbackOwnerTest {
    private void register() {
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.LOCAL_SOLVER);
        assertTrue(sync.registerForwardFlightEmergency(this, FlightControlAdmission.snapshot(sync), 0));
    }

    @Test void registeredHeadingCannotHijackHighNormalDescent() {
        supported = true;
        sync.position.set(new Vector3d(175, 110, -32));
        sync.fallDistance.set(2d);
        register();
        for (int i = 0; i < 12; i++) {
            sync.pitch.set(23f);
            motion.run();
            assertEquals(23f, sync.pitch.get());
            assertEquals(0, uses());
        }
        assertTrue(sync.position.get().y < 110);
    }

    @Test void registeredPoweredHeadingCannotClimbThroughRoof() {
        lava = true; ceiling = true;
        attach();
        register();
        for (int i = 0; i < 12; i++) {
            sync.position.set(new Vector3d(.5, 10, .5));
            sync.velocity.set(new Vector3d(0, -.08, 0));
            sync.pitch.set(23f);
            motion.run();
            assertEquals(23f, sync.pitch.get(), "roof sweep must reject even registered emergency");
            assertEquals(0, uses());
        }
    }

    @Test void alternatingCorrectionPointsCannotRenewAirborneBudget() {
        lava = true;
        for (int i = 0; i < 150; i++) {
            sync.invalidateFlightControls();
            sync.position.set(new Vector3d(i % 2 == 0 ? .5 : 32.5, 10, .5));
            sync.velocity.set(new Vector3d(0, -.08, 0));
            sync.pitch.set(17f);
            motion.run();
        }
        assertEquals(1, uses());
        assertEquals(17f, sync.pitch.get());
    }

    @Test void landingLabelCannotAuthorizeDamagingTouchdown() {
        supported = true;
        sync.position.set(new Vector3d(.5, 8, .5));
        sync.velocity.set(new Vector3d(0, -1, .4));
        sync.fallDistance.set(6d);
        register();
        var source = FlightControlAdmission.snapshot(sync);
        var dive = new ElytraFlightController.Decision(0, 85, false,
                ElytraFlightController.Phase.LANDING);
        sync.submitFlightControls(FlightControlAdmission.createPrefix(this, source,
                List.of(new FlightControlAdmission.Step(dive, source.state())), List.of(), null, () -> {}));
        sync.pitch.set(-5f);
        motion.run();
        assertEquals(-5f, sync.pitch.get());
        assertNotNull(sync.getFlightControlFailure());
        assertEquals(0, uses());
    }

    @Test void normalLandingCompletesThroughAdmissionInsteadOfClimbContinuation() {
        supported = true;
        sync.position.set(new Vector3d(.5, 8, .5));
        sync.velocity.set(new Vector3d(0, -.6, .4));
        sync.fallDistance.set(2d);
        register();
        int admitted = 0;
        for (int i = 0; i < 100 && !sync.onGround.get(); i++) {
            var source = FlightControlAdmission.snapshot(sync);
            var control = new ElytraFlightController.Decision(0, 23, false,
                    ElytraFlightController.Phase.LANDING);
            sync.submitFlightControls(FlightControlAdmission.createPrefix(this, source,
                    List.of(new FlightControlAdmission.Step(control, source.state())), List.of(), null, () -> {}));
            sync.pitch.set(-5f);
            motion.run();
            assertEquals(23f, sync.pitch.get(), "normal fixed landing must survive admission");
            assertEquals(0, uses());
            admitted++;
        }
        assertTrue(admitted > 6);
        assertTrue(sync.onGround.get(), "must actually complete, not only label an airborne phase LANDING");
        assertEquals(1, sync.position.get().y, .001);
    }
}
