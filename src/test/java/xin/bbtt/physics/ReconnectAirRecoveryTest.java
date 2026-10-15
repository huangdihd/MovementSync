package xin.bbtt.physics;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class ReconnectAirRecoveryTest {
    @Test void fallingIntoVoidNeedsDeployment() {
        assertTrue(ReconnectAirRecovery.needsDeployment(false, false, -.1, 0, true));
    }
    @Test void highFallNeedsDeploymentButShortFallDoesNot() {
        assertTrue(ReconnectAirRecovery.needsDeployment(false, false, -.1, 20, false));
        assertFalse(ReconnectAirRecovery.needsDeployment(false, false, -.1, 3, false));
    }
    @Test void preservesServerGlideAndDoesNotDeployWhileRisingOrGrounded() {
        assertFalse(ReconnectAirRecovery.needsDeployment(false, true, -1, 20, true));
        assertFalse(ReconnectAirRecovery.needsDeployment(false, false, 1, 20, true));
        assertFalse(ReconnectAirRecovery.needsDeployment(true, false, -1, 20, true));
    }
}
