package xin.bbtt.movements;

import org.junit.jupiter.api.Test;
import xin.bbtt.inventory.InventoryManager;
import xin.bbtt.physics.ElytraFlightController;

import static org.junit.jupiter.api.Assertions.assertEquals;

class VanillaGlideLifecyclePhysicalClockTest extends PhysicalClockTestSupport {
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
    void repeatedPlanningCannotConsumePhysicalCooldownOrBoostLifetime() {
        physicalTick();
        var lifecycle = queueRocket();
        for (int i = 0; i < 50; i++) {
            lifecycle.tickCooldown(movementSync);
            movementSync.publishPlacementFrame(0, 0, 0, 0, 0, 1.62);
        }
        assertEquals(40, lifecycle.rocketCooldown());
        assertEquals(21, lifecycle.confirmedBoostTicks(movementSync, true));
    }

    @Test
    void skippedPlanningChargesAllPhysicalTicksFromRequestNotClockOrigin() {
        for (int i = 0; i < 100; i++) physicalTick();
        var lifecycle = queueRocket();
        for (int i = 0; i < 7; i++) physicalTick();
        assertEquals(14, lifecycle.confirmedBoostTicks(movementSync, true));
        lifecycle.tickCooldown(movementSync);
        assertEquals(33, lifecycle.rocketCooldown());
        for (int i = 0; i < 40; i++) physicalTick();
        lifecycle.tickCooldown(movementSync);
        assertEquals(0, lifecycle.rocketCooldown());
        assertEquals(1, lifecycle.confirmedBoostTicks(movementSync, true));
    }

    @Test
    void requestBeforeFirstPhysicalTickChargesRequestToConfirmationLatency() {
        var lifecycle = queueRocket();
        for (int i = 0; i < 7; i++) physicalTick();
        lifecycle.tickCooldown(movementSync);
        assertEquals(33, lifecycle.rocketCooldown());
        assertEquals(14, lifecycle.confirmedBoostTicks(movementSync, true));
        assertEquals(14, lifecycle.confirmedBoostTicks(movementSync, true));
        assertEquals(0, lifecycle.confirmedBoostTicks(movementSync, false));
        assertEquals(1, lifecycle.confirmedBoostTicks(movementSync, true));
    }
}
