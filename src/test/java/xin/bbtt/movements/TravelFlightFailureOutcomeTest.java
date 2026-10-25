package xin.bbtt.movements;

import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** A failed flight leg cannot be rewritten as arrival by a later walking leg. */
final class TravelFlightFailureOutcomeTest {
    @Test void failedFlightAbortsOriginalGotoWithoutStartingWalking() throws Exception {
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
        sync.setGliding(true);
        Vector3i goal = new Vector3i(400, 64, 0);
        long generation = sync.beginStaticNavigationRequest(goal);
        int[] plans = {0};
        int[] walks = {0};
        int[] flights = {0};
        ElytraFlyMovement flight = new ElytraFlyMovement(new Vector3d(400.5, 64, .5), generation);
        TravelMovement travel = new TravelMovement(goal, generation, true, new TravelMovement.Legs() {
            @Override public List<PathStep> plan(Vector3i destination) {
                plans[0]++;
                return List.of();
            }
            @Override public Movement walk(List<PathStep> path, Vector3i destination, boolean intermediate) {
                walks[0]++;
                throw new AssertionError("failed flight must not start walking");
            }
            @Override public Movement flight() {
                flights[0]++;
                return flight;
            }
        });

        travel.init();
        travel.onTick();
        assertEquals(1, flights[0]);
        assertFalse(flight.isFinished(), "parent must initialize a live flight child");
        assertEquals(goal, sync.getActiveGoal());
        assertEquals(0, plans[0]);
        assertEquals(0, walks[0]);

        sync.setGliding(false);
        travel.onTick();

        assertTrue(flight.isFinished());
        assertEquals(ElytraFlyMovement.Completion.FAILED, flight.completion());
        assertTrue(travel.isFinished());
        assertEquals(1, sync.aborts);
        assertEquals(0, sync.arrivals, "failed flight must never announce destination arrival");
        assertEquals(0, plans[0], "failed flight must not plan a masking walking leg");
        assertEquals(0, walks[0]);
        assertNull(sync.getActiveGoal());
        assertEquals("flight leg failed before destination arrival", sync.reason);
    }
}
