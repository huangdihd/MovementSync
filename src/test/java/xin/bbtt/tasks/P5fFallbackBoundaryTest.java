package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.*;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.physics.FlightControlAdmission;
import xin.bbtt.movements.ElytraFlyMovement;

/** Real input-boundary motion, packet requests and physical displacement. */
class P5fFallbackBoundaryTest extends P5eFallbackOwnerTest {
    @Test void normalHighLandingNeverBecomesHazardClimb() {
        supported = true;
        sync.position.set(new Vector3d(175, 110, -32));
        sync.fallDistance.set(2d);
        sync.claimFlightControlOwner(this, FlightControlAdmission.Mode.LOCAL_SOLVER);
        for (int i = 0; i < 12; i++) {
            sync.pitch.set(23f);
            motion.run();
            assertEquals(23f, sync.pitch.get());
            assertEquals(0, uses());
            assertFalse(sync.getFlightControlFailure().reason().startsWith("HARD_FALLBACK"));
        }
        assertTrue(sync.position.get().y < 110);
    }
    @Test void distantLavaIsNotAnEmergencySurface() {
        lava = true;
        sync.position.set(new Vector3d(.5, 96, .5));
        sync.pitch.set(23f);
        motion.run();
        assertEquals(23f, sync.pitch.get());
        assertEquals(0, uses());
        assertTrue(sync.position.get().y < 96);
    }
    @Test void landingOwnerWithoutEnvelopeIsNotOverridden() {
        lava = true;
        var landing = new ElytraFlyMovement(new Vector3d(.5, 1, .5)).forIndependentLanding();
        sync.claimFlightControlOwner(landing, FlightControlAdmission.Mode.DIRECT);
        sync.pitch.set(23f);
        motion.run();
        assertEquals(23f, sync.pitch.get());
        assertEquals(0, uses());
    }
    @Test void risingAttachmentDoesNotKeepEmergencyClimbOwnership() {
        lava = true;
        attach();
        sync.velocity.set(new Vector3d(0, .4, .2));
        sync.pitch.set(12f);
        motion.run();
        assertEquals(12f, sync.pitch.get());
        assertEquals(0, uses());
    }
    @Test void stationaryCorrectionsCannotRenewBoostBudget() {
        lava = true;
        for (int i = 0; i < 150; i++) {
            sync.invalidateFlightControls();
            sync.position.set(new Vector3d(.5, 10, .5));
            sync.velocity.set(new Vector3d(0, -.08, 0));
            sync.pitch.set(17f);
            motion.run();
        }
        assertEquals(1, uses());
        assertEquals(17f, sync.pitch.get(), "bounded episode must release even at the same hazard");
    }
    @Test void roofNeverAuthorizesPoweredFallback() {
        lava = true; ceiling = true;
        for (int i = 0; i < 20; i++) {
            sync.position.set(new Vector3d(.5, 10, .5));
            sync.velocity.set(new Vector3d(0, -.08, 0));
            motion.run();
            assertEquals(0, uses());
        }
    }
}
