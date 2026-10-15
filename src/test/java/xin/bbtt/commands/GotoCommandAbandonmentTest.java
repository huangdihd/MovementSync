package xin.bbtt.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.world.World;

final class GotoCommandAbandonmentTest {
    @Test
    void noPathClearsActiveGoalWithoutAdvancingGeneration() {
        MovementSync movementSync = new MovementSync();

        new GotoCommandExecutor().onCommand(null, "goto", new String[] {"0", "0", "0"});

        assertEquals(1, movementSync.getNavigationGeneration());
        assertNull(movementSync.getActiveGoal());
    }

    @Test
    void pathfindingExceptionClearsActiveGoalWithoutAdvancingGeneration() {
        boolean[] failedAfterBegin = {false};
        MovementSync movementSync = new MovementSync() {
            @Override
            public World getWorld() {
                if (getNavigationGeneration() > 0) {
                    failedAfterBegin[0] = true;
                    throw new IllegalStateException("injected pathfinding failure");
                }
                return super.getWorld();
            }
        };

        new GotoCommandExecutor().onCommand(null, "goto", new String[] {"4", "0", "0"});

        assertTrue(failedAfterBegin[0]);
        assertEquals(1, movementSync.getNavigationGeneration());
        assertNull(movementSync.getActiveGoal());
    }
}
