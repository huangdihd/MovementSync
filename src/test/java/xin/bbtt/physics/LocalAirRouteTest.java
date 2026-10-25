package xin.bbtt.physics;

import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class LocalAirRouteTest {
    private record Box(double x, double y, double z, double X, double Y, double Z) {
        boolean overlaps(double a, double b, double c, double A, double B, double C) {
            return a < X && A > x && b < Y && B > y && c < Z && C > z;
        }
    }

    private static World terrain(Box... boxes) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) {
                for (Box box : boxes) if (box.overlaps(x, y, z, X, Y, Z)) return true;
                return false;
            }
        };
    }

    private static PhysicsState state(Vector3d p, double speed) {
        return new PhysicsState(p, new Vector3d(speed, 0, 0), false, 0, PlayerPose.GLIDING);
    }

    private static LocalAirRoute route(Vector3d... points) {
        return new LocalAirRoute(new WaypointFlightTarget(List.of(points), points[points.length - 1], false));
    }

    private static boolean visible(World world, Vector3d a, Vector3d b) {
        return LocalAirRoute.visible(world, a, b, PlayerPose.GLIDING);
    }

    @Test void snapshotStartsAtTargetsCurrentSegment() {
        Vector3d a = new Vector3d(0, 5, 0), b = new Vector3d(10, 5, 0);
        Vector3d c = new Vector3d(20, 5, 0), d = new Vector3d(30, 5, 0);
        var target = new WaypointFlightTarget(List.of(a, b, c, d), d, false);
        assertTrue(target.reconcile(terrain(), state(new Vector3d(25, 5, 0), 1)));
        assertEquals(3, target.index());

        var route = new LocalAirRoute(target);

        assertEquals(a.distance(b) + b.distance(c), route.cursor(), 1.0e-6,
                "a worker snapshot must not steer back onto consumed segments");
    }

    @Test void liveSegmentRenewalAdvancesButNeverRewindsWorkerArcProgress() {
        Vector3d a = new Vector3d(0, 5, 0), b = new Vector3d(10, 5, 0);
        Vector3d c = new Vector3d(20, 5, 0), d = new Vector3d(30, 5, 0);
        var target = new WaypointFlightTarget(List.of(a, b, c, d), d, false);
        var route = new LocalAirRoute(target);
        route.lookahead(terrain(), state(new Vector3d(15, 5, 0), 1));
        assertEquals(15, route.cursor(), 1.0e-6);

        assertTrue(target.reconcile(terrain(), state(new Vector3d(25, 5, 0), 1)));
        route.advanceToLiveSegment(target);
        assertEquals(20, route.cursor(), 1.0e-6);

        var behind = new WaypointFlightTarget(List.of(a, b, c, d), d, false);
        route.advanceToLiveSegment(behind);
        assertEquals(20, route.cursor(), 1.0e-6);
    }

    @Test void snapshotAndReturnedVectorsCannotMutateGeometry() {
        Vector3d start = new Vector3d(0, 5, 0), end = new Vector3d(30, 5, 0);
        WaypointFlightTarget target = new WaypointFlightTarget(List.of(start, end), end, false);
        LocalAirRoute route = new LocalAirRoute(target);
        start.set(999);
        end.set(999);
        target.waypoints().get(0).set(999);
        route.waypoints().get(0).set(999);
        List<Vector3d> first = route.lookahead(terrain(), state(new Vector3d(0, 5, 0), 1));
        Vector3d expected = new Vector3d(first.get(0));
        first.get(0).set(999);
        assertEquals(expected, route.lookahead(terrain(), state(new Vector3d(0, 5, 0), 1)).get(0));
        assertEquals(new Vector3d(0, 5, 0), route.waypoints().get(0));
        assertThrows(UnsupportedOperationException.class, () -> first.add(new Vector3d()));
    }

    @Test void bendAroundWallDoesNotCutThroughIt() {
        World world = terrain(new Box(4, 0, -2, 6, 10, 8));
        Vector3d a = new Vector3d(0, 5, 0), b = new Vector3d(0, 5, 10);
        Vector3d c = new Vector3d(10, 5, 10), d = new Vector3d(10, 5, 0);
        LocalAirRoute route = route(a, b, c, d);
        assertFalse(visible(world, a, d));
        List<Vector3d> aims = route.lookahead(world, state(a, 3));
        assertFalse(aims.isEmpty());
        assertFalse(aims.contains(d));
        for (Vector3d aim : aims) assertTrue(visible(world, a, aim));
        route.lookahead(world, state(b, 3));
        assertTrue(route.cursor() >= 10);
        route.lookahead(world, state(c, 3));
        assertTrue(route.cursor() >= 20);
    }

    @Test void narrowDoorUsesBodyWidthAndPoseRatherThanCentreRay() {
        Vector3d a = new Vector3d(0, 2, 0), b = new Vector3d(8, 2, 0);
        World fits = terrain(new Box(3, 0, -10, 4, 10, -.4), new Box(3, 0, .4, 4, 10, 10),
                new Box(3, 2.8, -.4, 4, 10, .4));
        assertTrue(visible(fits, a, b));
        assertFalse(LocalAirRoute.visible(fits, a, b, PlayerPose.STANDING));
        World tooNarrow = terrain(new Box(3, 0, -10, 4, 10, -.29), new Box(3, 0, .29, 4, 10, 10));
        assertFalse(visible(tooNarrow, a, b));
    }

    @Test void zeroLengthStillChecksBodyAndUnknownChunk() {
        Vector3d p = new Vector3d(0, 5, 0);
        assertTrue(visible(terrain(), p, p));
        assertFalse(visible(terrain(new Box(.2, 5, -.1, .3, 6, .1)), p, p));
        assertFalse(visible(new World(), p, p));
        LocalAirRoute duplicate = route(p, p, p);
        assertEquals(List.of(p), duplicate.lookahead(terrain(), state(p, 0)));
        assertEquals(0, duplicate.cursor());
    }

    @Test void upperAndLowerPlatformsBlockVerticalShortcutsButAllowSideApproach() {
        World world = terrain(new Box(-2, 3, -2, 2, 4, 2), new Box(-2, 8, -2, 2, 9, 2));
        Vector3d below = new Vector3d(0, 1, 0), middle = new Vector3d(0, 5, 0);
        Vector3d above = new Vector3d(0, 10, 0), outside = new Vector3d(4, 5, 0);
        assertFalse(visible(world, below, middle));
        assertFalse(visible(world, above, middle));
        assertTrue(visible(world, outside, middle));
        assertTrue(visible(world, new Vector3d(4, 10, 0), above));
    }

    @Test void cornerTangentCannotHideBodyClipAndDiagonalEnvelopeIsRefined() {
        World world = terrain(new Box(2, 0, 2, 3, 10, 3));
        assertFalse(visible(world, new Vector3d(0, 5, 4), new Vector3d(4, 5, 0)),
                "centre ray touches a corner but the player's body clips it");
        assertTrue(visible(world, new Vector3d(0, 5, 3), new Vector3d(3, 5, 0)),
                "an enclosing AABB overlaps but the continuous body sweep clears the corner");
        World thin = terrain(new Box(1.12345, 0, -1, 1.12346, 10, 1));
        assertFalse(visible(thin, new Vector3d(0, 5, 0), new Vector3d(5, 5, 0)),
                "sub-sample thin obstacles must not be tunnelled through");
    }

    @Test void speedExtendsLookaheadAndUnloadedFootprintLimitsIt() {
        Vector3d a = new Vector3d(1, 5, 1), b = new Vector3d(45, 5, 1);
        double slow = route(a, b).lookahead(terrain(), state(a, .1)).get(0).x;
        double fast = route(a, b).lookahead(terrain(), state(a, 2)).get(0).x;
        assertTrue(fast > slow);
        World loaded = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return x == 0 && z == 0; }
        };
        List<Vector3d> aims = route(a, b).lookahead(loaded, state(a, 4));
        assertFalse(aims.isEmpty());
        assertTrue(aims.stream().allMatch(p -> p.x + PlayerPose.HALF_WIDTH <= 16));
        assertFalse(visible(loaded, new Vector3d(15.8, 5, 1), new Vector3d(15.8, 5, 1)));
    }

    @Test void longSyntheticTerrainRouteProjectsByArcAndNeverRewinds() {
        Vector3d start = new Vector3d(-3.5, 36, -1.5), end = new Vector3d(175, 60, -32);
        Vector3d[] points = {start, new Vector3d(35, 42, -1.5), new Vector3d(35, 46, -20),
                new Vector3d(90, 52, -20), new Vector3d(90, 56, -32), end};
        World world = terrain(new Box(-20, 0, -50, 200, 30, 20),
                new Box(40, 30, -15, 75, 70, 10), new Box(100, 30, -25, 140, 80, 5));
        LocalAirRoute route = route(points);
        double arc = 0;
        double previousAimArc = 0;
        for (int segment = 1; segment < points.length; segment++) {
            double length = points[segment - 1].distance(points[segment]);
            for (int step = 0; step <= 20; step++) {
                Vector3d p = new Vector3d(points[segment - 1]).lerp(points[segment], step / 20.0);
                List<Vector3d> aims = route.lookahead(world, state(p, 1.5));
                assertFalse(aims.isEmpty());
                assertEquals(arc + length * step / 20, route.cursor(), 1.0e-6);
                for (Vector3d aim : aims) assertTrue(visible(world, p, aim));
                double aimArc = arcOf(points, aims.get(0));
                assertTrue(aimArc + 1.0e-6 >= previousAimArc, "forward steering must not rewind");
                previousAimArc = aimArc;
                double before = route.cursor();
                route.lookahead(world, state(new Vector3d(p).sub(.01, 0, 0), 1.5));
                assertTrue(route.cursor() >= before);
            }
            arc += length;
        }
        assertEquals(route.length(), route.cursor(), 1.0e-6);
    }

    private static double arcOf(Vector3d[] points, Vector3d p) {
        double arc = 0;
        for (int i = 1; i < points.length; i++) {
            double length = points[i - 1].distance(points[i]);
            if (Math.abs(points[i - 1].distance(p) + p.distance(points[i]) - length) < 1.0e-6)
                return arc + points[i - 1].distance(p);
            arc += length;
        }
        fail("Aim is not on route: " + p);
        return 0;
    }
}
