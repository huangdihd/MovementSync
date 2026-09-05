package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

final class PathProgressWatchdogTest {
    @Test
    void abortsAfterConfiguredTicksWithoutAdvancingAPathNode() {
        PathProgressWatchdog watchdog = new PathProgressWatchdog(3);

        assertFalse(watchdog.tick());
        assertFalse(watchdog.tick());
        assertTrue(watchdog.tick());
    }

    @Test
    void advancingAPathNodeResetsTheWatchdog() {
        PathProgressWatchdog watchdog = new PathProgressWatchdog(3);

        assertFalse(watchdog.tick());
        assertFalse(watchdog.tick());
        watchdog.nodeAdvanced();
        assertFalse(watchdog.tick());
        assertFalse(watchdog.tick());
        assertTrue(watchdog.tick());
    }
}
