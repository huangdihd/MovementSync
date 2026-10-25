package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import xin.bbtt.inventory.InventoryManager;
import xin.bbtt.physics.ElytraFlightController;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Mechanism isolation, not a historical implementation replay or evidence of live causation.
 * Both variants retain today's execute/useFirework path and shared physical request guard.
 * Only the test variant replaces the local decision decrement with elapsed physical ticks.
 * Demand is fixed, packets are drained immediately, and no attachment is acknowledged.
 */
class VanillaGlideLifecycleClockIsolationTest extends PhysicalClockTestSupport {
    private enum Clock { CURRENT_DECISION, TEST_ONLY_PHYSICAL }

    private enum Schedule {
        ONE_DECISION_PER_TICK(1, 160),
        ONE_DECISION_PER_FOUR_TICKS(4, 40),
        REPEATED_DECISIONS_WITHOUT_PHYSICS(0, 120);

        final int ticksPerDecision;
        final int decisionsAfterInitial;

        Schedule(int ticksPerDecision, int decisionsAfterInitial) {
            this.ticksPerDecision = ticksPerDecision;
            this.decisionsAfterInitial = decisionsAfterInitial;
        }
    }

    private static final ElytraFlightController.Decision DEMAND =
            new ElytraFlightController.Decision(0, 0, true, ElytraFlightController.Phase.CRUISE);

    static Stream<Arguments> clocksAndSchedules() {
        return Stream.of(Schedule.values()).flatMap(schedule ->
                Stream.of(Clock.values()).map(clock -> Arguments.of(schedule, clock)));
    }

    @Test
    void sparseCruisePlanningCannotSpendRocketsFasterThanFortyDecisions() {
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        movementSync.setGliding(true);
        physicalTick();
        long origin = movementSync.physicsTickVersion();
        var lifecycle = new VanillaGlideLifecycle();
        List<Integer> dispatchDecisions = new ArrayList<>();
        List<Long> dispatchTicks = new ArrayList<>();
        long advanced = 0;

        // The valid 30k failure averaged about 11.5 decisions/s while physics stayed at 20 Hz.
        // Seven physical ticks per four decisions is a deterministic 11.43 decisions/s model.
        for (int decision = 0; decision <= 40; decision++) {
            long target = decision * 7L / 4L;
            while (advanced < target) {
                physicalTick();
                advanced++;
            }
            if (decision > 0) lifecycle.tickCooldown(movementSync);
            lifecycle.execute(movementSync, DEMAND, InventoryManager.ElytraAction.READY);
            if (dispatchQueuedFireworks() == 1) {
                dispatchDecisions.add(decision);
                dispatchTicks.add(movementSync.physicsTickVersion() - origin);
            }
        }

        assertEquals(List.of(0, 40), dispatchDecisions,
                "sparse planning must retain the established forty-decision cruise budget");
        assertEquals(List.of(0L, 70L), dispatchTicks);
        assertTrue(dispatchTicks.get(1) - dispatchTicks.get(0) >= 40,
                "the shared physical request latch remains authoritative");
    }

    @ParameterizedTest(name = "{0}: {1}")
    @MethodSource("clocksAndSchedules")
    void isolatesLocalClockWithTheSameCurrentSharedGuard(Schedule schedule, Clock clock) throws Exception {
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        movementSync.setGliding(true);
        // Establish a real physical driver before the initial request: tick zero has an
        // intentional embedded-caller fallback and is not the mechanism under test.
        physicalTick();
        long origin = movementSync.physicsTickVersion();
        assertTrue(origin > 0);
        var lifecycle = new VanillaGlideLifecycle();
        Field localCooldown = VanillaGlideLifecycle.class.getDeclaredField("rocketCooldown");
        localCooldown.setAccessible(true);
        List<Long> dispatchTicks = new ArrayList<>();
        long lastDispatchTick = -40;
        int lastDispatchDecision = 0;
        int sharedGuardRejections = 0;
        boolean readyWhileLocalCooling = false;

        for (int decision = 0; decision <= schedule.decisionsAfterInitial; decision++) {
            if (decision > 0) {
                for (int tick = 0; tick < schedule.ticksPerDecision; tick++) {
                    physicalTick();
                    long elapsed = movementSync.physicsTickVersion() - origin;
                    assertEquals(elapsed - lastDispatchTick >= 40, movementSync.fireworkRequestReady(),
                            "shared guard readiness follows physical time even without decisions");
                    assertNoConfirmedBoost(lifecycle);
                }
                int before = lifecycle.rocketCooldown();
                lifecycle.tickCooldown(movementSync);
                if (clock == Clock.TEST_ONLY_PHYSICAL) {
                    // Test-only counterfactual: preserve all current lifecycle behavior except
                    // this local decrement. Never replace or bypass the shared request guard.
                    localCooldown.setInt(lifecycle, Math.max(0, before - schedule.ticksPerDecision));
                }
            }

            long elapsed = movementSync.physicsTickVersion() - origin;
            assertEquals((long) decision * schedule.ticksPerDecision, elapsed);
            int expectedCooldown = decision == 0 ? 0 : (int) Math.max(0, 40 -
                    (clock == Clock.CURRENT_DECISION ? decision - lastDispatchDecision
                            : elapsed - lastDispatchTick));
            assertEquals(expectedCooldown, lifecycle.rocketCooldown(), "local clock before demand");
            assertCooldownBounds(lifecycle);
            boolean sharedReady = elapsed - lastDispatchTick >= 40;
            assertEquals(sharedReady, movementSync.fireworkRequestReady());
            boolean shouldDispatch = expectedCooldown == 0 && sharedReady;
            boolean rejectedBySharedGuard = expectedCooldown == 0 && !sharedReady;
            readyWhileLocalCooling |= sharedReady && expectedCooldown > 0;

            lifecycle.execute(movementSync, DEMAND, InventoryManager.ElytraAction.READY);
            int drainedUses = dispatchQueuedFireworks();
            assertEquals(shouldDispatch ? 1 : 0, drainedUses, "immediately dispatched use packets");
            assertNull(movementSync.pollBeforeMovementPacket(), "no unsent request remains queued");
            if (shouldDispatch) {
                dispatchTicks.add(elapsed);
                lastDispatchTick = elapsed;
                lastDispatchDecision = decision;
                assertEquals(40, lifecycle.rocketCooldown());
            } else {
                assertEquals(expectedCooldown, lifecycle.rocketCooldown(), "rejection must not reset local cooldown");
            }
            if (rejectedBySharedGuard) sharedGuardRejections++;
            assertEquals(elapsed - lastDispatchTick >= 40, movementSync.fireworkRequestReady(),
                    "draining a packet must retain the shared physical cooldown");
            assertCooldownBounds(lifecycle);
            assertNoConfirmedBoost(lifecycle);
            assertTrue(movementSync.isGliding());
            assertEquals(64, inventory[36].getAmount(), "inventory is server-authoritative");
        }

        List<Long> expectedDispatches = switch (schedule) {
            case ONE_DECISION_PER_TICK -> List.of(0L, 40L, 80L, 120L, 160L);
            case ONE_DECISION_PER_FOUR_TICKS -> clock == Clock.CURRENT_DECISION
                    ? List.of(0L, 160L) : List.of(0L, 40L, 80L, 120L, 160L);
            case REPEATED_DECISIONS_WITHOUT_PHYSICS -> List.of(0L);
        };
        assertEquals(expectedDispatches, dispatchTicks);
        for (int i = 1; i < dispatchTicks.size(); i++) {
            assertTrue(dispatchTicks.get(i) - dispatchTicks.get(i - 1) >= 40,
                    "neither local clock may bypass the shared physical bound");
        }
        boolean decisionClockWithoutPhysics = schedule == Schedule.REPEATED_DECISIONS_WITHOUT_PHYSICS
                && clock == Clock.CURRENT_DECISION;
        assertEquals(decisionClockWithoutPhysics ? 81 : 0, sharedGuardRejections,
                "decisions 40 through 120 reach local zero but the shared guard rejects all 81");
        assertEquals(decisionClockWithoutPhysics ? 0 : 40, lifecycle.rocketCooldown());
        assertFalse(movementSync.fireworkRequestReady());
        assertEquals(schedule == Schedule.ONE_DECISION_PER_FOUR_TICKS
                && clock == Clock.CURRENT_DECISION, readyWhileLocalCooling,
                "sparse decisions keep the local counter closed after the shared guard is ready");
    }

    private void assertNoConfirmedBoost(VanillaGlideLifecycle lifecycle) {
        assertFalse(movementSync.hasAttachedFirework(), "requests do not fabricate attachments");
        assertEquals(0, lifecycle.confirmedBoostTicks(movementSync, movementSync.hasAttachedFirework()),
                "neither local clock establishes thrust without an attachment");
    }

    private static void assertCooldownBounds(VanillaGlideLifecycle lifecycle) {
        assertTrue(lifecycle.rocketCooldown() >= 0 && lifecycle.rocketCooldown() <= 40,
                "local cooldown stays in [0, 40]");
    }
}
