package xin.bbtt.physics;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class ReturnResourceCheckpointTest {
    @Test void requestsAreSeparateFromPowerAndRecordedExactlyOnce() {
        var schedule = FireworkSchedule.confirmed(new int[] {1, 3}, 2, 0, 0, 2);
        var first = schedule.nextTick();
        assertTrue(first.useRequested());
        assertFalse(first.powered());
        assertEquals(1, first.checkpoint().consumed());
        assertEquals(1, first.checkpoint().pendingTicks());
        assertFalse(schedule.nextTick().powered());
        var confirmed = schedule.nextTick();
        assertFalse(confirmed.useRequested());
        assertTrue(confirmed.powered());
        assertEquals(-1, confirmed.checkpoint().pendingTicks());
        assertEquals(18, confirmed.checkpoint().boostTicks());
        int requests = 1;
        for (int i = 3; i < 100; i++) if (schedule.nextTick().useRequested()) requests++;
        assertEquals(2, requests);
    }

    @Test void suppressedUsesAgePendingBoostWithoutConsumingAnotherRocket() {
        var schedule = FireworkSchedule.confirmed(new int[] {1, 1}, 2, 0, 0, 3);
        schedule.nextTick();
        int powered = 0;
        for (int i = 0; i < 100; i++) {
            var step = schedule.nextTick(false);
            assertFalse(step.useRequested());
            if (step.powered()) powered++;
        }
        assertEquals(18, powered);
        assertEquals(1, schedule.checkpoint().consumed());
        assertEquals(0, schedule.checkpoint().cooldownTicks());
        assertEquals(-1, schedule.checkpoint().pendingTicks());
    }

    @Test void immutableSnapshotAndLegacyClockShareTransitions() {
        int[] inventory = {1, 3};
        var events = FireworkSchedule.confirmed(inventory, 2, 7, 4, 2);
        var legacy = FireworkSchedule.confirmed(inventory, 2, 7, 4, 2);
        inventory[0] = 255;
        var checkpoint = events.checkpoint();
        for (int i = 0; i < 100; i++) assertEquals(legacy.nextPowered(), events.nextTick().powered());
        assertEquals(4, checkpoint.boostTicks());
        assertEquals(7, checkpoint.cooldownTicks());
        assertEquals(0, checkpoint.consumed());
        assertEquals(legacy.checkpoint(), events.checkpoint());
    }
}
