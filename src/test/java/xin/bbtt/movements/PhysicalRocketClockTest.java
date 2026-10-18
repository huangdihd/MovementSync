package xin.bbtt.movements;

import org.junit.jupiter.api.Test;
import xin.bbtt.inventory.InventoryManager;
import xin.bbtt.physics.ElytraFlightController;
import static org.junit.jupiter.api.Assertions.assertEquals;

class PhysicalRocketClockTest extends PhysicalClockTestSupport {
    @Test void planningCallsDoNotConsumeConfirmedRocketLife() {
        inventory[36] = item("firework_rocket", 2);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        movementSync.setGliding(true);
        physicalTick();
        var lifecycle = new VanillaGlideLifecycle();
        lifecycle.execute(movementSync, new ElytraFlightController.Decision(0, 0, true,
                ElytraFlightController.Phase.CRUISE), InventoryManager.ElytraAction.READY);
        for (int i = 0; i < 8; i++) lifecycle.tickCooldown(movementSync);
        assertEquals(40, lifecycle.rocketCooldown());
        assertEquals(21, lifecycle.confirmedBoostTicks(movementSync, true));
        for (int i = 0; i < 4; i++) physicalTick();
        assertEquals(17, lifecycle.confirmedBoostTicks(movementSync, true));
        lifecycle.tickCooldown(movementSync);
        assertEquals(36, lifecycle.rocketCooldown());
    }
}
