package xin.bbtt.movements;

import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.world.World;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/** The return shore is not a waypoint on a walking route across the original hazard. */
final class TravelPlannedReturnOutcomeTest {
    @Test void groundedReserveReturnAbortsOriginalGotoWithoutPlanningWalking() throws Exception {
        BlockState stone = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        BlockState air = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
        World terrain = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 64 ? stone : air; }
        };
        class Sync extends MovementSync {
            int arrivals;
            int aborts;
            String reason;
            @Override public World getWorld() { return terrain; }
            @Override public boolean isGliding() { return false; }
            @Override public void setActiveGoal(Vector3i goal) {
                if (goal == null) arrivals++;
                super.setActiveGoal(goal);
            }
            @Override public void abandonNavigationRequest(long generation, String why) {
                aborts++;
                reason = why;
                super.abandonNavigationRequest(generation, why);
            }
        }
        Sync sync = new Sync();
        sync.position.set(new Vector3d(.5, 64, .5));
        sync.velocity.set(new Vector3d());
        sync.onGround.set(true);
        Vector3i goal = new Vector3i(400, 64, 0);
        long generation = sync.beginStaticNavigationRequest(goal);
        TravelMovement travel = new TravelMovement(goal, generation, true, new TravelMovement.Legs() {
            @Override public List<PathStep> plan(Vector3i destination) {
                fail("planned return must not search a walking route to the original ocean goal");
                return List.of();
            }
            @Override public Movement walk(List<PathStep> path, Vector3i destination, boolean intermediate) {
                throw new AssertionError("planned return must not start walking");
            }
            @Override public Movement flight() { throw new AssertionError("must not relaunch"); }
        });
        ElytraFlyMovement flight = new ElytraFlyMovement(new Vector3d(400.5, 64, .5), generation);
        var returnSite = ElytraFlyMovement.class.getDeclaredField("reserveReturnSite");
        returnSite.setAccessible(true);
        returnSite.set(flight, new Vector3d(sync.position.get()));
        var finish = ElytraFlyMovement.class.getDeclaredMethod("finish", boolean.class);
        finish.setAccessible(true);
        finish.invoke(flight, false);
        assertTrue(flight.isFinished(), "fixture is an observed grounded return completion");
        assertEquals(goal, sync.getActiveGoal(), "flight completion is not original goto arrival");
        var child = TravelMovement.class.getDeclaredField("child");
        child.setAccessible(true);
        child.set(travel, flight);
        var stage = TravelMovement.class.getDeclaredField("stage");
        stage.setAccessible(true);
        for (Object value : stage.getType().getEnumConstants()) {
            if (value.toString().equals("FLIGHT")) stage.set(travel, value);
        }
        travel.onTick();
        travel.onTick();
        assertTrue(travel.isFinished());
        assertEquals(1, sync.aborts);
        assertEquals(0, sync.arrivals, "return landing must never announce original destination arrival");
        assertNull(sync.getActiveGoal());
        assertTrue(sync.reason.contains("planned reserve return"), sync.reason);
    }
}
