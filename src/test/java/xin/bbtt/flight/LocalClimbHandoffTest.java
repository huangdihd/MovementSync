package xin.bbtt.flight;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.physics.PhysicsState;
import static org.junit.jupiter.api.Assertions.*;

class LocalClimbHandoffTest {
    @Test void requiresTenDistinctConsecutivePhysicalObservations() {
        var handoff = new LocalClimbHandoff();
        handoff.deployed(70);
        for (int tick = 100; tick < 109; tick++) {
            assertFalse(handoff.observe(tick, true));
            for (int duplicate = 0; duplicate < 20; duplicate++)
                assertFalse(handoff.observe(tick, true));
        }
        assertEquals(9, handoff.stableTicks());
        assertTrue(handoff.observe(109, true));
        assertEquals(LocalClimbHandoff.Phase.CRUISE, handoff.phase());
        assertTrue(handoff.observe(110, false), "Cruise cannot oscillate back into initial climb");
    }

    @Test void interruptedOrUnobservedClearanceRestartsHysteresis() {
        var handoff = new LocalClimbHandoff();
        handoff.deployed(70);
        for (int tick = 1; tick <= 8; tick++) handoff.observe(tick, true);
        assertFalse(handoff.observe(9, false));
        assertEquals(0, handoff.stableTicks());
        assertFalse(handoff.observe(30, true));
        assertEquals(1, handoff.stableTicks());
        assertFalse(handoff.observe(32, true));
        assertEquals(1, handoff.stableTicks());
        assertFalse(handoff.observe(32, false));
        assertEquals(0, handoff.stableTicks());
    }

    @Test void geometricClimbUsesDepartureHeightNotDestinationHeight() {
        var handoff = new LocalClimbHandoff();
        handoff.deployed(70);
        var state = new PhysicsState(new Vector3d(0, 72, 0), new Vector3d(0, .2, 1), false);
        var route = handoff.route(state, new Vector3d(0, 31, 1000), 0);
        assertEquals(24, route.goal().z, 1.0e-9);
        assertEquals(86, route.goal().y, 1.0e-9);
        assertEquals(4, route.corridorRadius(), 1.0e-9);
        assertFalse(route.landed());
    }

    @Test void inheritedCruiseNeverStartsInitialClimb() {
        var handoff = new LocalClimbHandoff();
        handoff.inheritAirborne();
        handoff.deployed(70);
        assertEquals(LocalClimbHandoff.Phase.CRUISE, handoff.phase());
    }

    @Test void climbRoutingExcludesEndPluginAndIsIndependentOfFollow() {
        var router = new ScopeRouter();
        assertEquals(ScopeRouter.Route.LOCAL, router.route(ScopeRouter.Scope.LOCAL_CLIMB, false, false));
        assertEquals(ScopeRouter.Route.LEGACY, router.route(ScopeRouter.Scope.LOCAL_CLIMB, true, false));
        assertEquals(ScopeRouter.Route.LEGACY, router.route(ScopeRouter.Scope.LOCAL_CLIMB, false, true));
        assertEquals(ScopeRouter.Route.LOCAL, router.route(ScopeRouter.Scope.LOCAL_FOLLOW, false, false));
        router.setLocalGotoEnabled(false);
        assertEquals(ScopeRouter.Route.LEGACY, router.route(ScopeRouter.Scope.LOCAL_CLIMB, false, false));
    }
}
