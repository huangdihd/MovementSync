package xin.bbtt.movements;

import java.util.Map;
import java.util.concurrent.TimeUnit;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Planner/deployment regression, not a substitute for live controller integration. */
final class LowAltitudeUnloadedDestinationTest {
    private static final class RoofWorld extends World {
        private final BlockState stone = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        int lastLoadedZ = 3;
        @Override public boolean chunkLoaded(int x, int z) {
            return x >= -1 && x <= 1 && z >= -1 && z <= lastLoadedZ;
        }
        @Override public BlockState getBlockStateAt(Vector3d p) {
            return p.y < 64 || p.y >= 80 ? stone : super.getBlockStateAt(p);
        }
    }

    @Test void stationaryLaunchUnderRoofAcceptsUnloadedSixHundredBlockGoal() throws Exception {
        RoofWorld world = new RoofWorld();
        Vector3d goal = new Vector3d(8, 64, 608);
        PhysicsState grounded = new PhysicsState(new Vector3d(8, 64, 8), new Vector3d(), true);
        assertFalse(world.chunkLoaded(0, 38));
        var future = LowAltitudeFlightPlan.submit(world, grounded, goal, MovementParameters.DEFAULT, false);
        var result = future.get(30, TimeUnit.SECONDS);
        assertNotNull(result);
        assertEquals(LowAltitudeFlightPlan.Status.PARTIAL, future.status());
        assertEquals(goal, result.destination());
        assertEquals(goal, result.route().goal());
        assertFalse(result.route().landed());
        for (Vector3d point : result.route().waypoints()) {
            assertTrue(world.chunkLoaded((int) Math.floor(point.x) >> 4, (int) Math.floor(point.z) >> 4));
            assertTrue(point.y >= 64 && point.y + PlayerPose.GLIDING.height() < 80);
        }
    }

    @Test void newlyLoadedChunksExtendPartialFixedRouteUntilRealLandingGoal() throws Exception {
        RoofWorld world = new RoofWorld();
        Vector3d goal = new Vector3d(8, 64, 608);
        PhysicsState state = new PhysicsState(new Vector3d(8, 66, 8), new Vector3d(), false);
        boolean complete = false;
        int partials = 0;
        for (int iteration = 0; iteration < 16; iteration++) {
            var future = LowAltitudeFlightPlan.submit(world, state, goal, MovementParameters.DEFAULT, true);
            var result = future.get(30, TimeUnit.SECONDS);
            assertNotNull(result, "Loaded corridor should remain plannable");
            var route = result.route();
            assertEquals(goal, route.goal());
            Vector3d end = route.waypoints().get(route.waypoints().size() - 1);
            assertTrue(end.z > state.position().z, "Every refreshed route must progress");
            assertTrue(end.z < (world.lastLoadedZ + 1) * 16);
            if (!route.partial()) {
                assertEquals(goal, end);
                assertEquals(LowAltitudeFlightPlan.Status.COMPLETE, future.status());
                complete = true;
                break;
            }
            partials++;
            assertFalse(route.landed(), "A loaded frontier is not the final landing");
            assertNotEquals(goal, end);
            // Simulate new terrain delivery and a new airborne planning snapshot.
            // Actual controller traversal is covered by movement-level tests.
            state = new PhysicsState(end, new Vector3d(), false);
            world.lastLoadedZ += 4;
        }
        assertTrue(partials > 1);
        assertTrue(complete, "Rolling loaded windows must eventually reach the fixed goal");
    }

    @Test void diagnosticDistinguishesInvalidInputFromUnsafeDeployment() throws Exception {
        RoofWorld world = new RoofWorld();
        PhysicsState grounded = new PhysicsState(new Vector3d(8, 64, 8), new Vector3d(), true);
        var invalid = LowAltitudeFlightPlan.submit(world, grounded, new Vector3d(Double.NaN, 64, 608),
                MovementParameters.DEFAULT, false);
        assertNull(invalid.get(30, TimeUnit.SECONDS));
        assertEquals(LowAltitudeFlightPlan.Status.INVALID_COORDINATES, invalid.status());
        World unknown = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
        };
        var rejected = LowAltitudeFlightPlan.submit(unknown, grounded, new Vector3d(8, 64, 608),
                MovementParameters.DEFAULT, false);
        assertNull(rejected.get(30, TimeUnit.SECONDS));
        assertEquals(LowAltitudeFlightPlan.Status.DEPLOYMENT_REJECTED, rejected.status());
    }
}
