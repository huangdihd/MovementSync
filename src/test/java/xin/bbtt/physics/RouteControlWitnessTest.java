package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Geometry never substitutes for a resource-aware terminal control proof. */
final class RouteControlWitnessTest {
    private static HazardRecovery.Context resources(int cooldown) {
        int[] rockets = new int[64];
        java.util.Arrays.fill(rockets, 1);
        return new HazardRecovery.Context(rockets, cooldown, false, null);
    }

    @Test void exhaustedSearchCanStillRevalidateControlsWithoutReusingSafety() {
        World world = HazardRecoveryPerformanceTest.sea();
        var state = new PhysicsState(new Vector3d(64, 48, .5), new Vector3d(.8, 0, 0),
                false, 0, PlayerPose.GLIDING);
        assertTrue(HazardRecovery.assess(world, state, MovementParameters.DEFAULT, resources(0)).admissible());
        var witness = HazardRecovery.controlWitness();
        assertNotNull(witness);
        assertFalse(HazardRecovery.assess(world, state, MovementParameters.DEFAULT, resources(0),
                new HazardRecovery.Budget(0)).admissible());
        var replay = new HazardRecovery.Budget(1200);
        assertTrue(HazardRecovery.revalidate(world, state, MovementParameters.DEFAULT, resources(0), witness,
                replay).admissible());
        assertTrue(replay.steps() <= 1200);
        assertFalse(HazardRecovery.revalidate(new World(), state, MovementParameters.DEFAULT, resources(0),
                witness, new HazardRecovery.Budget(1200)).admissible());
        assertFalse(HazardRecovery.revalidate(world, state, MovementParameters.DEFAULT, resources(0),
                witness, new HazardRecovery.Budget(0)).admissible());
        var low = new PhysicsState(new Vector3d(64, 32.31, .5), new Vector3d(), false, 0, PlayerPose.GLIDING);
        assertFalse(HazardRecovery.revalidate(world, low, MovementParameters.DEFAULT, HazardRecovery.Context.EMPTY,
                witness, new HazardRecovery.Budget(1200)).admissible());
        assertFalse(HazardRecovery.revalidate(world, low, MovementParameters.DEFAULT, resources(40),
                witness, new HazardRecovery.Budget(1200)).admissible());
    }

    @Test void narrowMidrouteWallInvalidatesWitnessDuringCooldown() {
        World clear = HazardRecoveryPerformanceTest.sea();
        var state = new PhysicsState(new Vector3d(64, 48, .5), new Vector3d(.8, 0, 0),
                false, 0, PlayerPose.GLIDING);
        var context = resources(10);
        var proof = HazardRecovery.assess(clear, state, MovementParameters.DEFAULT, context);
        assertEquals(HazardRecovery.Status.RECOVERABLE, proof.status());
        var witness = HazardRecovery.controlWitness();
        assertTrue(HazardRecovery.revalidate(clear, state, MovementParameters.DEFAULT, context,
                witness, new HazardRecovery.Budget(1200)).admissible());
        double wallX = Math.floor((state.position().x + proof.target().x) / 2);
        var stone = new xin.bbtt.Block.BlockState("minecraft:stone", 1, java.util.Map.of(),
                "block", 1.5, true, "STONE");
        World blocked = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return clear.chunkLoaded(x, z); }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d p) {
                return p.x >= wallX && p.x < wallX + 1 && p.y >= 32 && p.y < 60
                        ? stone : clear.getBlockStateAt(p);
            }
        };
        var budget = new HazardRecovery.Budget(1200);
        assertFalse(HazardRecovery.revalidate(blocked, state, MovementParameters.DEFAULT, context,
                witness, budget).admissible(), "old controls cannot authorize crossing a newly placed wall");
        assertTrue(budget.steps() <= 1200);
    }

    @Test void sampledStatesRequireFreshTerminalProof() {
        double[][] samples = {{42.13, 58.84, -8.181, .7342, .5509, -.002954},
                {100.7, 52.53, -10.02, 1.294, -.1485, -.127}};
        for (double[] s : samples) {
            var state = new PhysicsState(new Vector3d(s[0], s[1], s[2]), new Vector3d(s[3], s[4], s[5]),
                    false, 0, PlayerPose.GLIDING);
            var budget = new HazardRecovery.Budget(HazardRecovery.DECISION_STEP_BUDGET);
            var result = HazardRecovery.assess(HazardRecoveryPerformanceTest.sea(), state,
                    MovementParameters.DEFAULT, resources(0), budget);
            assertTrue(budget.steps() <= HazardRecovery.DECISION_STEP_BUDGET);
            if (result.status() == HazardRecovery.Status.RECOVERABLE) {
                assertTrue(HazardRecovery.revalidate(HazardRecoveryPerformanceTest.sea(), state,
                        MovementParameters.DEFAULT, resources(0), HazardRecovery.controlWitness(),
                        new HazardRecovery.Budget(1200)).admissible());
            }
        }
    }
}
