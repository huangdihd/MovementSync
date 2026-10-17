package xin.bbtt.recovery;

import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import static org.junit.jupiter.api.Assertions.*;

class FollowCorrectionTest {
    @Test void ordinaryServerCorrectionsDoNotCancelFollowOrReplaceItWithIndependentLanding() {
        var sync = new MovementSync();
        sync.onRecoveryTeleport();
        sync.setFollowTargetId(42);
        long generation = sync.getNavigationGeneration();
        for (int i = 0; i < 3; i++) {
            sync.onRecoveryTeleport();
            assertEquals(generation, sync.getNavigationGeneration());
            assertEquals(42, sync.getFollowTargetId());
            assertNull(sync.getMovementController().getCurrentMovement());
        }
        sync.cancelNavigation();
        assertEquals(-1, sync.getFollowTargetId());
        assertFalse(sync.isNavigationRequestCurrent(generation));
    }
}
