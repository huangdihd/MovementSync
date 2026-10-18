package xin.bbtt.commands;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import static org.junit.jupiter.api.Assertions.*;

class DebugLocalFollowCommandTest {
    @AfterEach void tearDown() { MovementSync.INSTANCE = null; }

    @Test void togglesIndependentlyAndInvalidatesFlightEpoch() {
        var sync = new MovementSync();
        var executor = new DebugCommandExecutor();
        long epoch = sync.flightControlEpoch();
        executor.onCommand(null, "debug", new String[] {"localfollow", "off"});
        assertFalse(sync.getFlightScopeRouter().isLocalFollowEnabled());
        assertTrue(sync.getFlightScopeRouter().isLocalGotoEnabled());
        assertTrue(sync.flightControlEpoch() > epoch);
        executor.onCommand(null, "debug", new String[] {"localgoto", "off"});
        epoch = sync.flightControlEpoch();
        executor.onCommand(null, "debug", new String[] {"LOCALFOLLOW", "ON"});
        assertTrue(sync.getFlightScopeRouter().isLocalFollowEnabled());
        assertFalse(sync.getFlightScopeRouter().isLocalGotoEnabled());
        assertTrue(sync.flightControlEpoch() > epoch);
    }

    @Test void rejectsInvalidToggleWithoutChangingFlagsOrEpoch() {
        var sync = new MovementSync();
        var executor = new DebugCommandExecutor();
        for (boolean enabled : new boolean[] {true, false}) {
            sync.getFlightScopeRouter().setLocalFollowEnabled(enabled);
            long epoch = sync.flightControlEpoch();
            executor.onCommand(null, "debug", new String[] {"localfollow", "invalid"});
            assertEquals(enabled, sync.getFlightScopeRouter().isLocalFollowEnabled());
            assertTrue(sync.getFlightScopeRouter().isLocalGotoEnabled());
            assertEquals(epoch, sync.flightControlEpoch());
        }
    }
}
