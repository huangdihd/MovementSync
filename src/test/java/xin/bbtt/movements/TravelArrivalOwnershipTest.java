package xin.bbtt.movements;

import java.util.List;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;

import static org.junit.jupiter.api.Assertions.*;

final class TravelArrivalOwnershipTest {
    @Test void childCompletionCannotAnnounceArrivalBeforeParentAbortsUnsupportedArrival() throws Exception {
        verifyCompletion(false);
    }

    @Test void supportedArrivalIsPublishedExactlyOnceByParent() throws Exception {
        verifyCompletion(true);
    }

    private void verifyCompletion(boolean supported) throws Exception {
        class Sync extends MovementSync {
            int arrivals;
            int aborts;
            @Override public boolean isGliding() { return false; }
            @Override public void setActiveGoal(Vector3i goal) {
                if (goal == null) arrivals++;
                super.setActiveGoal(goal);
            }
            @Override public void abandonNavigationRequest(long generation, String reason) {
                aborts++;
                super.abandonNavigationRequest(generation, reason);
            }
        }
        Sync sync = new Sync();
        Vector3i goal = new Vector3i(175, 60, -32);
        long generation = sync.beginStaticNavigationRequest(goal);
        sync.position.set(new Vector3d(175.5, 60, -31.5));
        sync.onGround.set(supported);
        TravelMovement travel = new TravelMovement(goal, generation, false);
        PathMovement child = new PathMovement(List.of(), -1, false, goal, -1, generation) {
            @Override public void init() {}
            @Override public void onTick() {}
            @Override public void onStop() {}
        };
        var stage = TravelMovement.class.getDeclaredField("stage");
        stage.setAccessible(true);
        for (Object value : stage.getType().getEnumConstants()) {
            if (value.toString().equals("WALK")) stage.set(travel, value);
        }
        var start = TravelMovement.class.getDeclaredMethod("startChild", Movement.class);
        start.setAccessible(true);
        start.invoke(travel, child);
        var finish = PathMovement.class.getDeclaredMethod("markFinished");
        finish.setAccessible(true);
        finish.invoke(child);
        assertEquals(goal, sync.getActiveGoal(), "child completion must not clear recoverable static intent");
        assertEquals(0, sync.arrivals);
        travel.onTick();
        travel.onTick();
        assertTrue(travel.isFinished());
        assertEquals(supported ? 1 : 0, sync.arrivals);
        assertEquals(supported ? 0 : 1, sync.aborts);
    }
}
