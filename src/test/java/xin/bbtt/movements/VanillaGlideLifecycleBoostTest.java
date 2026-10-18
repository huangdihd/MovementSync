package xin.bbtt.movements;

import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.inventory.InventoryManager;
import xin.bbtt.physics.ElytraFlightController;

import static org.junit.jupiter.api.Assertions.assertEquals;

class VanillaGlideLifecycleBoostTest extends ElytraCommandTestSupport {
    private VanillaGlideLifecycle queueRocket() {
        inventory[36] = item("firework_rocket", 2);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        movementSync.setGliding(true);
        var lifecycle = new VanillaGlideLifecycle();
        lifecycle.execute(movementSync, new ElytraFlightController.Decision(0, 0, true,
                ElytraFlightController.Phase.CRUISE), InventoryManager.ElytraAction.READY);
        assertEquals(40, lifecycle.rocketCooldown());
        return lifecycle;
    }

    @Test
    void queuedRocketNeedsConfirmationAndChargesLatency() {
        var lifecycle = queueRocket();
        assertEquals(0, lifecycle.confirmedBoostTicks(movementSync, false));
        for (int i = 0; i < 4; i++) lifecycle.tickCooldown(movementSync);
        assertEquals(17, lifecycle.confirmedBoostTicks(movementSync, true));
        assertEquals(17, lifecycle.confirmedBoostTicks(movementSync, true));
        assertEquals(0, lifecycle.confirmedBoostTicks(movementSync, false));
        assertEquals(1, lifecycle.confirmedBoostTicks(movementSync, true));
    }

    @Test
    void physicsTicksChargeLatencyEvenWhenFlightActionDoesNotRun() {
        var lifecycle = queueRocket();
        for (int i = 0; i < 7; i++) movementSync.publishPlacementFrame(0, 0, 0, 0, 0, 1.62);
        assertEquals(14, lifecycle.confirmedBoostTicks(movementSync, true));
        for (int i = 0; i < 30; i++) movementSync.publishPlacementFrame(0, 0, 0, 0, 0, 1.62);
        assertEquals(1, lifecycle.confirmedBoostTicks(movementSync, true));
    }

    @Test
    void inheritedRocketHasOnlyCurrentTickGuarantee() {
        var lifecycle = new VanillaGlideLifecycle();
        assertEquals(1, lifecycle.confirmedBoostTicks(movementSync, true));
        assertEquals(0, lifecycle.confirmedBoostTicks(movementSync, false));
    }
}
