package xin.bbtt.physics;

import org.junit.jupiter.api.Test;
import xin.bbtt.movements.TravelMovement;
import static org.junit.jupiter.api.Assertions.*;

class ElytraDurabilityTest {
    @Test void excludesLastPointAndDoesNotCreditUnbreaking() {
        assertEquals(11, new ElytraDurability(true, 432, 420, false).remainingFlightSeconds());
        assertFalse(new ElytraDurability(true, 432, 431, false).usable());
        assertEquals(0, new ElytraDurability(false, 432, 0, false).remainingFlightSeconds());
    }
    @Test void highLandingRequiresEarlierAction() {
        var durability = new ElytraDurability(true, 432, 410, false);
        assertFalse(durability.shouldLand());
        assertTrue(durability.shouldLand(240));
        assertEquals(30, ElytraDurability.landingSeconds(240));
    }
    @Test void plannerRejectsInsufficientBudgetAndWalkingNeverFlies() {
        var worn = new ElytraDurability(true, 432, 420, false);
        var fresh = new ElytraDurability(true, 432, 0, false);
        assertFalse(TravelMovement.shouldFly(600, worn, true, false, false));
        assertTrue(TravelMovement.shouldFly(600, fresh, true, false, false));
        assertFalse(TravelMovement.shouldFly(600, fresh, true, true, false));
    }
}
