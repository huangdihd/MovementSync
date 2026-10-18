package xin.bbtt.movements;

import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.inventory.InventoryManager;
import xin.bbtt.physics.ElytraFlightController;
import static org.junit.jupiter.api.Assertions.*;

class VanillaRocketFailureTest {
    @Test void repeatedRescueRequestsCannotBypassExecutionCooldown() {
        int[] uses = {0};
        var sync = new MovementSync() {
            @Override public boolean hasAttachedFirework() { return false; }
            @Override public boolean useFirework() { uses[0]++; return true; }
        };
        var lifecycle = new VanillaGlideLifecycle();
        var decision = new ElytraFlightController.Decision(0, -10, true,
                ElytraFlightController.Phase.LANDING);
        for (int tick = 0; tick < 200; tick++) {
            lifecycle.tickCooldown(sync);
            lifecycle.execute(sync, decision, InventoryManager.ElytraAction.READY);
        }
        assertEquals(5, uses[0], "ten seconds of fallback requests must not consume a stack");
    }

    @Test void rejectedRocketDoesNotInventThrustOrPreventNextRescueAttempt() {
        var sync = new MovementSync() {
            @Override public boolean hasAttachedFirework() { return false; }
            @Override public boolean useFirework() { return false; }
        };
        var lifecycle = new VanillaGlideLifecycle();
        lifecycle.execute(sync, new ElytraFlightController.Decision(0, -55, true,
                ElytraFlightController.Phase.AVOIDANCE), InventoryManager.ElytraAction.READY);
        assertEquals(0, lifecycle.rocketCooldown());
    }
}
