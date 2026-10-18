package xin.bbtt.physics;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Sampled decision workload, without running the 400-tick integration scenario. */
final class HazardRecoveryPerformanceTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0, Map.of(), "empty", 0, false, "LAVA");
    static World sea() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return x >= -2 && x <= 12 && z >= -2 && z <= 2; }
            @Override public java.util.Set<ChunkPosition> loadedChunks() {
                var chunks = new java.util.HashSet<ChunkPosition>();
                for (int x = -2; x <= 12; x++) for (int z = -2; z <= 2; z++) chunks.add(new ChunkPosition(x, z));
                return java.util.Set.copyOf(chunks);
            }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y >= 60 || p.y < 29 || p.x >= 0 && p.x < 16 && Math.abs(p.z) < 8 && p.y < 32
                        || p.x >= 128 && p.y < 42) return STONE;
                return p.y < 32 ? LAVA : AIR;
            }
        };
    }
    @Test void sampledPoweredReserve() {
        World world = sea();
        int[] rockets = new int[64];
        java.util.Arrays.fill(rockets, 1);
        var context = new HazardRecovery.Context(rockets, 0, false, null);
        long start = System.nanoTime();
        var budget = new HazardRecovery.Budget(HazardRecovery.DECISION_STEP_BUDGET);
        var report = HazardRecovery.assess(world, new PhysicsState(new Vector3d(64, 48, .5),
                new Vector3d(.8, 0, 0), false, 0, PlayerPose.GLIDING), MovementParameters.DEFAULT, context, budget);
        System.out.printf("Sampled powered reserve: %.3f ms (%s)%n", (System.nanoTime() - start) / 1.e6, report.status());
        assertEquals(HazardRecovery.Status.RECOVERABLE, report.status());
        assertTrue(budget.steps() <= HazardRecovery.DECISION_STEP_BUDGET);
    }

    @Test void exhaustedBudgetNeverReusesPriorSafety() {
        var state = new PhysicsState(new Vector3d(64, 48, .5), new Vector3d(.8, 0, 0), false, 0, PlayerPose.GLIDING);
        int[] rockets = new int[64];
        java.util.Arrays.fill(rockets, 1);
        var context = new HazardRecovery.Context(rockets, 0, false, null);
        assertTrue(HazardRecovery.assess(sea(), state, MovementParameters.DEFAULT, context).admissible());
        var budget = new HazardRecovery.Budget(1);
        assertFalse(HazardRecovery.assess(sea(), state, MovementParameters.DEFAULT, context, budget).admissible());
        assertEquals(1, budget.steps());
        assertFalse(HazardRecovery.assess(new World(), state, MovementParameters.DEFAULT, context).admissible());
        var low = new PhysicsState(new Vector3d(64, 32.31, .5), new Vector3d(), false, 0, PlayerPose.GLIDING);
        assertFalse(HazardRecovery.assess(sea(), low, MovementParameters.DEFAULT, HazardRecovery.Context.EMPTY).admissible(),
                "a remembered powered control cannot credit spent rockets");
    }

    @Test void sampledActualSelectorDecision() {
        var state = new PhysicsState(new Vector3d(64, 48, .5), new Vector3d(.8, 0, 0), false, 0, PlayerPose.GLIDING);
        var goal = new Vector3d(130, 48, .5);
        var route = new WaypointFlightTarget(java.util.List.of(state.position(), goal), goal, false);
        long start = System.nanoTime();
        var decision = new FlightSelector().decide(sea(), state, -90, 0, false, route, 64, 2);
        System.out.printf("Sampled actual low decision: %.3f ms%n", (System.nanoTime() - start) / 1.e6);
        assertNotNull(decision);
    }
}
