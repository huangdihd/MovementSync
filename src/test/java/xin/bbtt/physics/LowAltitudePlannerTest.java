package xin.bbtt.physics;

import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class LowAltitudePlannerTest {
    private final World air = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
    };
    private final PhysicsState initial = new PhysicsState(new Vector3d(8, 100, 8), new Vector3d(), false);

    @Test void pursuitReservesSteeringDropAboveUpcomingHumpNotOnlyEndpointColumn() {
        BlockState stone = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        World rolling = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return p.y < 64 || p.x < -65 && p.x >= -69 && p.y < 67
                        ? stone : super.getBlockStateAt(p);
            }
        };
        PhysicsState flying = new PhysicsState(new Vector3d(-54.5, 68, 28.5),
                new Vector3d(-.8, -.2, 0), false);
        Vector3d endpoint = new Vector3d(-61.5, 64, 28.5);
        var route = LowAltitudePlanner.planPursuit(rolling, flying, endpoint, MovementParameters.DEFAULT);
        assertNotNull(route);
        assertTrue(route.pursuit());
        assertFalse(route.landed());
        assertTrue(route.goal().y > 67 + PlayerPose.GLIDING.height(),
                "Upcoming terrain must retain an airborne steering reserve: " + route.goal());
        assertEquals(endpoint, new Vector3d(-61.5, 64, 28.5));
    }

    @Test void geometricRouteReachesDistantGoalAndCopiesVectors() {
        Vector3d goal = new Vector3d(8, 100, 408);
        var route = LowAltitudePlanner.plan(air, initial, goal, MovementParameters.DEFAULT);
        assertNotNull(route);
        assertFalse(route.partial());
        assertEquals(goal, route.waypoints().get(route.waypoints().size() - 1));
        assertTrue(route.waypoints().size() < 10, "Straight route should be LOS-smoothed");
        goal.zero();
        route.waypoints().get(0).zero();
        assertEquals(initial.position(), route.waypoints().get(0));
        assertEquals(new Vector3d(8, 100, 408), route.goal());
    }

    @Test void unloadedGoalReturnsLoadedFrontierInsteadOfRejectingDistance() {
        World limited = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return x == 0 && z >= 0 && z < 4; }
        };
        var route = LowAltitudePlanner.plan(limited, initial, new Vector3d(8, 100, 1000), MovementParameters.DEFAULT);
        assertNotNull(route);
        assertTrue(route.partial());
        Vector3d end = route.waypoints().get(route.waypoints().size() - 1);
        assertTrue(end.z > 50 && end.z < 64);
        assertFalse(route.landed());
        assertNotEquals(route.goal(), end);
    }

    @Test void unknownStartHazardsAndGroundedStartAreBlocked() {
        World unknown = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
        };
        World fire = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return new BlockState("fire", 0, Map.of(), "empty", 0, false, "");
            }
        };
        Vector3d goal = new Vector3d(8, 100, 20);
        assertNull(LowAltitudePlanner.plan(unknown, initial, goal, MovementParameters.DEFAULT));
        assertNull(LowAltitudePlanner.plan(fire, initial, goal, MovementParameters.DEFAULT));
        assertNull(LowAltitudePlanner.plan(air, new PhysicsState(initial.position(), new Vector3d(), true),
                goal, MovementParameters.DEFAULT));
    }

    @Test void lavaAndSubBodyWidthPassageAreNotAirCorridors() {
        World lava = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return new BlockState("minecraft:lava", 0, Map.of(), "empty", 0, false, "");
            }
        };
        World narrow = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return x < 7.71 || X > 8.29; }
        };
        assertNull(LowAltitudePlanner.plan(lava, initial, new Vector3d(8, 100, 30), MovementParameters.DEFAULT));
        assertNull(LowAltitudePlanner.plan(narrow, initial, new Vector3d(8, 100, 30), MovementParameters.DEFAULT));
    }

    @Test void exactFloatBodyRejectsSliverAndSweptLosRejectsThinWall() {
        World wall = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return X > 1 && x < 1.001; }
        };
        assertFalse(LowAltitudePlanner.lineOfSight(wall, new Vector3d(.7, 3, 0), new Vector3d(.7, 3, 0), 0, 0));
        assertFalse(LowAltitudePlanner.lineOfSight(wall, new Vector3d(0, 3, 0), new Vector3d(3, 3, 0), 0, 0));
    }

    @Test void corridorDoesNotAcceptChordAcrossCornerAndAimDoesNotAdvance() {
        var route = new WaypointFlightTarget(List.of(new Vector3d(0, 100, 0),
                new Vector3d(0, 100, 10), new Vector3d(10, 100, 10)), new Vector3d(10, 100, 10), false, false, .5);
        int index = route.index();
        route.aim(initial);
        assertEquals(index, route.index());
        assertTrue(route.contains(new Vector3d(0, 100, 1), new Vector3d(0, 100, 9)));
        assertFalse(route.contains(new Vector3d(0, 100, 1), new Vector3d(9, 100, 10)));
        assertFalse(route.advance(air, initial, MovementParameters.DEFAULT));
    }

    @Test void threeDimensionalGridDetoursAroundObstruction() {
        World obstacle = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) {
                return x < 11 && X > 5 && y < 103 && Y > 97 && z < 20 && Z > 16;
            }
        };
        var route = LowAltitudePlanner.plan(obstacle, initial, new Vector3d(8, 100, 30), MovementParameters.DEFAULT);
        assertNotNull(route);
        assertFalse(route.partial());
        assertTrue(route.waypoints().size() > 2);
        List<Vector3d> points = route.waypoints();
        for (int i = 1; i < points.size(); i++)
            assertTrue(LowAltitudePlanner.lineOfSight(obstacle, points.get(i - 1), points.get(i), 0, 0));
    }

    @Test void advancementRequiresSafePhysicalReachability() {
        PhysicsState moving = new PhysicsState(new Vector3d(8, 100, 8), new Vector3d(0, 0, 1), false);
        PhysicsState predicted = moving;
        for (int i = 0; i < 6; i++) predicted = FlightSafety.nextState(new GlidingPhysics(0, 0, false)
                .step(air, predicted, LowAltitudePlanner.HALF_WIDTH, LowAltitudePlanner.HEIGHT));
        var route = new WaypointFlightTarget(List.of(moving.position(), moving.position().add(0, 0, 2),
                predicted.position()), predicted.position(), false);
        assertTrue(route.advance(air, moving, MovementParameters.DEFAULT));
        assertEquals(2, route.index());
        var impossible = new WaypointFlightTarget(List.of(moving.position(), moving.position().add(0, 0, 2),
                moving.position().add(0, 40, 0)), moving.position().add(0, 40, 0), false);
        assertFalse(impossible.advance(air, moving, MovementParameters.DEFAULT));
    }

    @Test void safeGroundEndpointIsAllowed() {
        World ground = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return y < 0; }
        };
        var route = LowAltitudePlanner.plan(ground,
                new PhysicsState(new Vector3d(8, 4, 8), new Vector3d(), false),
                new Vector3d(8, 0, 20), MovementParameters.DEFAULT);
        assertNotNull(route);
        assertFalse(route.partial());
        assertEquals(new Vector3d(8, 0, 20), route.waypoints().get(route.waypoints().size() - 1));
    }

    @Test void geometryExpansionAllowsDriftCurvesButNotRoofOrWall() {
        World enclosed = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return y < 0 || Y > 3 || X > 10 || x < 6; }
        };
        var route = LowAltitudePlanner.plan(enclosed,
                new PhysicsState(new Vector3d(8, 1, 8), new Vector3d(0, 0, .8), false),
                new Vector3d(8, 1, 40), MovementParameters.DEFAULT);
        assertNotNull(route, "Deployment and .8 speed must not demand three-block margins");
        assertTrue(route.contains(new Vector3d(8.4, 1, 8), new Vector3d(9, .8, 12)));
        assertFalse(route.contains(new Vector3d(8, 1, 8), new Vector3d(9.8, 1, 12)));
        assertFalse(route.contains(new Vector3d(8, 1, 8), new Vector3d(8, 2.5, 12)));
    }

    @Test void landingVolumeAllowsLateralDriftButNeverBelowSupport() {
        World ground = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return y < 0; }
        };
        var route = LowAltitudePlanner.plan(ground,
                new PhysicsState(new Vector3d(8, 1, 8), new Vector3d(0, 0, .8), false),
                new Vector3d(8, 0, 16), MovementParameters.DEFAULT);
        assertNotNull(route);
        assertFalse(route.partial());
        assertTrue(route.contains(new Vector3d(8.4, .5, 15), new Vector3d(8.4, 0, 16)));
        assertFalse(route.contains(new Vector3d(8.4, .5, 15), new Vector3d(8.4, -.01, 16)));
    }

    @Test void crossedCornerAdvancesTowardDistantNextWithStagedPreview() {
        var route = new WaypointFlightTarget(List.of(new Vector3d(8, 100, 0),
                new Vector3d(8, 100, 8), new Vector3d(8, 100, 180)),
                new Vector3d(8, 100, 180), false, false, 6, air);
        assertTrue(route.advance(air, new PhysicsState(new Vector3d(8.4, 100, 9),
                new Vector3d(0, 0, .8), false), MovementParameters.DEFAULT));
        assertEquals(2, route.index());
    }

    @Test void asynchronousSearchUsesWorker() throws Exception {
        Thread caller = Thread.currentThread();
        World workerWorld = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                assertNotSame(caller, Thread.currentThread());
                return true;
            }
        };
        assertNotNull(LowAltitudePlanner.planAsync(workerWorld, initial, new Vector3d(8, 100, 20),
                MovementParameters.DEFAULT).get(10, TimeUnit.SECONDS));
    }
}
