package xin.bbtt.physics;

import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class KnownLandingReserveTest {
    @Test void resultDefensivelyCopiesSiteAndThresholdIncludesMargin() {
        Vector3d site = new Vector3d(1, 2, 3);
        var result = new KnownLandingReserve.Result(KnownLandingReserve.Status.VALID,
                site, 2, 1, 0, true, true);
        site.zero();
        result.returnSite().zero();
        assertEquals(new Vector3d(1, 2, 3), result.returnSite());
        assertTrue(result.shouldReturn());
        assertFalse(result.canContinue());
    }

    @Test void absentKnownGroundCannotAuthorizeExposure() {
        var result = KnownLandingReserve.assess(new World(),
                new PhysicsState(new Vector3d(0, 20, 0), new Vector3d(), false),
                MovementParameters.DEFAULT, HazardRecovery.Context.EMPTY, List.of(), 2, 1);
        assertEquals(KnownLandingReserve.Status.NO_SITE, result.status());
        assertTrue(result.resourcesValid());
        assertFalse(result.geometryValid());
        assertTrue(result.shouldReturn());
        assertEquals(-1, result.requiredRockets());
    }

    @Test void invalidAcknowledgementBoundCannotAuthorizeExposure() {
        var result = KnownLandingReserve.assess(new World(),
                new PhysicsState(new Vector3d(0, 20, 0), new Vector3d(), false),
                MovementParameters.DEFAULT, HazardRecovery.Context.EMPTY, List.of(), -1, 1);
        assertEquals(KnownLandingReserve.Status.INVALID_RESOURCES, result.status());
        assertFalse(result.canContinue());
    }
}
