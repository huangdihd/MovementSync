package xin.bbtt.physics;

import java.util.List;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Round 13 coordinates are inputs, not evidence of historical worker ordering. */
final class LowRouteCursorRegressionTest {
    private final World air = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
    };
    private final World wall = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public boolean isBoxColliding(double x, double y, double z,
                double X, double Y, double Z) { return x < 161 && X > 160; }
    };

    @Test void publicationSelectsLiveSegmentInsteadOfResettingToFirstVertex() {
        var a = new Vector3d(70, 56.08, -9.893);
        var b = new Vector3d(81.56, 56.08, -9.893);
        var c = new Vector3d(150, 52.55, -19.80);
        var d = new Vector3d(159, 52.55, -19.80);
        var route = new WaypointFlightTarget(List.of(a, b, c, d), d, false, true, 6, air);
        var state = new PhysicsState(new Vector3d(158.6, 52.55, -19.80), new Vector3d(0, .3434, .06377), false);
        assertEquals(1, route.index());
        assertTrue(route.reconcile(air, state));
        assertEquals(3, route.index());
        assertEquals(d, route.position());
        assertTrue(route.partial());
        assertFalse(route.landed(), "a partial endpoint never becomes a landing goal");
        assertTrue(route.reconcile(air, state));
        assertEquals(3, route.index(), "repeated reconciliation must not reset progress");
    }

    @Test void closerOutgoingCapsuleCannotReconcileAcrossUnpassedBend() {
        var corner = new Vector3d(0, 5, 10);
        var end = new Vector3d(10, 5, 10);
        var route = new WaypointFlightTarget(List.of(new Vector3d(0, 5, 0), corner, end),
                end, false, false, 6, air);
        var before = new PhysicsState(new Vector3d(1, 5, 9.9), new Vector3d(0, 0, .2), false);
        assertTrue(route.reconcile(air, before));
        assertEquals(1, route.index(), "capsule tolerance must not skip an unpassed corner");
        assertEquals(corner, route.position());
        var after = new PhysicsState(new Vector3d(1, 5, 10.1), before.velocity(), false);
        assertTrue(route.reconcile(air, after));
        assertEquals(2, route.index(), "observed passage may reconcile onto the outgoing segment");
    }

    @Test void proximityAcrossX160WallCannotPublishOrSkip() {
        var route = new WaypointFlightTarget(List.of(new Vector3d(150, 56.07, -18.19),
                new Vector3d(163, 56.07, -18.19)), new Vector3d(163, 56.07, -18.19), false, false, 6, wall);
        var state = new PhysicsState(new Vector3d(159.7, 56.07, -18.19), new Vector3d(0, .3434, .06377), false);
        assertFalse(route.reconcile(wall, state));
        assertEquals(1, route.index());
    }

    @Test void offCorridorDiagnosticCannotAuthorizeOldVertexSteering() {
        var route = new WaypointFlightTarget(List.of(new Vector3d(70, 56.08, -9.893),
                new Vector3d(81.56, 56.08, -9.893)), new Vector3d(81.56, 56.08, -9.893), false, true, 6, wall);
        var state = new PhysicsState(new Vector3d(158.6, 52.55, -19.80), new Vector3d(0, .3434, .06377), false);
        assertFalse(route.reconcile(wall, state));
        assertThrows(IllegalStateException.class, () -> new FlightSelector().decide(wall, state,
                -87.49f, -10, false, route, 0, 2));
        assertEquals(1, route.index());
    }

    @Test void firstWaypointLineOfSightDoesNotLicenseSkippingBlockedBend() {
        var route = new WaypointFlightTarget(List.of(new Vector3d(155, 56, -20),
                new Vector3d(158, 56, -20), new Vector3d(164, 56, -20)),
                new Vector3d(164, 56, -20), false, false, 6, wall);
        assertFalse(route.advance(wall, new PhysicsState(new Vector3d(158, 56, -20),
                new Vector3d(.8, 0, 0), false), MovementParameters.DEFAULT));
        assertEquals(1, route.index());
    }
}
