package xin.bbtt.movements;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.MovementParameters;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.world.World;

import java.util.Map;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.*;

/** Exercises the actual worker, jump preview and corridor search, not injected outcomes. */
final class LoadedLowAltitudeDeploymentRegressionTest {
    private static final BlockState STONE = new BlockState(
            "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
            "minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");

    private static World corridor(int roof) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return p.y < 64 || p.y >= roof || p.x < 0 || p.x >= 3 ? STONE : AIR;
            }
        };
    }

    @Test void loaded125And155BlockGoalsUnderLowRoofRejectActualDeployment() throws Exception {
        World world = corridor(66);
        PhysicsState ground = new PhysicsState(new Vector3d(1.5, 64, .5), new Vector3d(), true);
        for (int distance : new int[] {125, 155}) {
            Vector3d goal = new Vector3d(1.5, 64, distance + .5);
            assertTrue(world.chunkLoaded(0, distance >> 4));
            var future = LowAltitudeFlightPlan.submit(world, ground, goal, MovementParameters.DEFAULT, false);
            try {
                assertNull(future.get(20, TimeUnit.SECONDS));
                assertEquals(LowAltitudeFlightPlan.Status.DEPLOYMENT_REJECTED, future.status(),
                        "loaded terrain must not be mislabeled as a missing chunk or equipment failure");
            } finally {
                future.cancel(true);
            }
        }
    }

    @Test void loaded130BlockGoalWithHeadroomProducesRealLaunchAndCorridor() throws Exception {
        World world = corridor(80);
        PhysicsState ground = new PhysicsState(new Vector3d(1.5, 64, .5), new Vector3d(), true);
        Vector3d goal = new Vector3d(1.5, 64, 130.5);
        assertTrue(world.chunkLoaded(0, 130 >> 4));
        PhysicsState launch = LowAltitudeFlightPlan.deployment(world, ground, MovementParameters.DEFAULT, 0, 0);
        assertNotNull(launch);
        assertFalse(launch.onGround());
        assertTrue(launch.position().y > ground.position().y);
        assertTrue(launch.velocity().y <= 0, "deployment follows the actual jump apex");
        var future = LowAltitudeFlightPlan.submit(world, ground, goal, MovementParameters.DEFAULT, false);
        try {
            var plan = future.get(20, TimeUnit.SECONDS);
            assertNotNull(plan);
            assertTrue(future.status() == LowAltitudeFlightPlan.Status.COMPLETE
                    || future.status() == LowAltitudeFlightPlan.Status.PARTIAL);
            assertEquals(ground.position(), plan.origin());
            assertEquals(goal, plan.destination());
            assertFalse(plan.route().waypoints().isEmpty());
            assertTrue(plan.route().waypoints().stream().anyMatch(point -> point.z > ground.position().z));
        } finally {
            future.cancel(true);
        }
    }
}
