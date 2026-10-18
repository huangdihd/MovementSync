package xin.bbtt.flight;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/** Clock/state-machine tests only; physical flight is exercised separately. */
class P4LocalClimbHandoffRegressionTest {
    @Test void repeatedPlannerCallsCannotManufactureTenPhysicalTicks() {
        var handoff = new LocalClimbHandoff();
        handoff.deployed(34.25);
        for (int i = 0; i < 100; i++) assertFalse(handoff.observe(7, true));
        assertEquals(1, handoff.stableTicks());
        assertEquals(LocalClimbHandoff.Phase.LOCAL_CLIMB, handoff.phase());
    }

    @Test void missingObservationOrLostClearanceRestartsStableWindow() {
        var handoff = new LocalClimbHandoff();
        handoff.deployed(37.25);
        for (int tick = 1; tick <= 9; tick++) assertFalse(handoff.observe(tick, true));
        assertFalse(handoff.observe(11, true));
        assertEquals(1, handoff.stableTicks());
        assertFalse(handoff.observe(11, false));
        assertEquals(0, handoff.stableTicks());
        for (int tick = 12; tick <= 20; tick++) assertFalse(handoff.observe(tick, true));
        assertTrue(handoff.observe(21, true));
    }

    @Test void cruiseHandoffIsOneWayEvenWhenLaterClearanceIsLost() {
        var handoff = new LocalClimbHandoff();
        assertEquals(LocalClimbHandoff.Phase.DEPLOYMENT, handoff.phase());
        handoff.deployed(34.25);
        for (int tick = 0; tick < 9; tick++) assertFalse(handoff.observe(tick, true));
        assertTrue(handoff.observe(9, true));
        assertEquals(LocalClimbHandoff.Phase.CRUISE, handoff.phase());
        handoff.deployed(70);
        assertTrue(handoff.observe(10, false));
        assertEquals(LocalClimbHandoff.Phase.CRUISE, handoff.phase());
    }
}
