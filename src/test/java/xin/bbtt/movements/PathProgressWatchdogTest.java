package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;
import xin.bbtt.pathfinding.Node;

final class PathProgressWatchdogTest {
    private final AtomicLong clock = new AtomicLong();
    private final PathProgressWatchdog watchdog = new PathProgressWatchdog(3, clock::get);

    @Test
    void abortsAfterConfiguredExecutionTimeWithoutAdvancingAPathNode() {
        assertFalse(watchdog.tick());
        assertFalse(nextTick());
        assertFalse(nextTick());
        assertTrue(nextTick());
    }

    @Test
    void advancingANewPathNodeResetsTheWatchdog() {
        watchdog.tick();
        nextTick();
        nextTick();
        watchdog.nodeAdvanced(new Node(1, 64, 0));
        assertFalse(nextTick());
        assertFalse(nextTick());
        assertTrue(nextTick());
    }

    @Test
    void repeatedStartNodeCompletionDoesNotHideInPlaceReplanning() {
        watchdog.tick();
        watchdog.nodeAdvanced(new Node(0, 64, 0));
        assertFalse(nextTick());
        watchdog.nodeAdvanced(new Node(0, 64, 0));
        assertFalse(nextTick());
        watchdog.nodeAdvanced(new Node(0, 64, 0));
        assertTrue(nextTick());
    }

    @Test
    void plannerTimeAndFixedRateCatchUpDoNotCountAsStalledExecution() {
        watchdog.tick();
        assertFalse(nextTick());
        clock.addAndGet(30_000_000_000L);
        watchdog.excludeElapsedTime();
        for (int index = 0; index < 600; index++) {
            assertFalse(watchdog.tick());
        }
        assertFalse(nextTick());
        assertTrue(nextTick());
    }

    @Test
    void schedulerDelayCountsAtMostOneMovementOpportunity() {
        watchdog.tick();
        clock.addAndGet(30_000_000_000L);
        assertFalse(watchdog.tick());
        assertFalse(nextTick());
        assertTrue(nextTick());
    }

    @Test
    void pauseExcludesInsertedMovementTimeWithoutResettingStallHistory() {
        watchdog.tick();
        assertFalse(nextTick());
        watchdog.pause();
        clock.addAndGet(30_000_000_000L);
        assertFalse(watchdog.tick());
        assertFalse(nextTick());
        assertTrue(nextTick());
    }

    private boolean nextTick() {
        clock.addAndGet(50_000_000L);
        return watchdog.tick();
    }
}
