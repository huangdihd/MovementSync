package xin.bbtt.physics;

import java.util.List;
import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class WaypointFlightControllerTest {
    private static final class Terrain extends World {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public boolean isBoxColliding(double x, double y, double z,
                double X, double Y, double Z) { return y < 0; }
        @Override public OptionalDouble findHighestCollisionTopExcluding(double x, double y, double z,
                double X, double Y, double Z, Set<Vector3i> excluded) {
            return y <= 0 && Y >= 0 ? OptionalDouble.of(0) : OptionalDouble.empty();
        }
    }

    @Test void rejectingBestCandidateStillConsidersSafeAlternatives() {
        var world = new Terrain();
        var route = new WaypointFlightTarget(List.of(new Vector3d(0, 5, 0), new Vector3d(0, 5, 40)),
                new Vector3d(0, 5, 40), false, false, 3);
        var state = new PhysicsState(new Vector3d(0, 5, 0), new Vector3d(0, 0, .1), false);
        var decision = new ElytraFlightController().decide(world, state, 0, 0, false,
                route, 4, 1, candidate -> !candidate.firework());
        assertFalse(decision.firework());
        assertEquals(ElytraFlightController.Phase.CRUISE, decision.phase());
    }

    @Test void driftReevaluatesControlsAndCanUseAvailableRocket() {
        var world = new Terrain();
        var route = new WaypointFlightTarget(List.of(new Vector3d(0, 5, 0), new Vector3d(0, 5, 40)),
                new Vector3d(0, 5, 40), false, false, 3);
        var controller = new ElytraFlightController();
        var original = new PhysicsState(new Vector3d(0, 5, 0), new Vector3d(0, 0, .1), false);
        var first = controller.decide(world, original, 0, 0, false, route, 4, 1);
        assertTrue(first.firework(), "Low-speed corridor pursuit should score available acceleration");
        var perturbed = new PhysicsState(new Vector3d(.4, 5.2, 1), new Vector3d(.1, -.02, .4), false);
        var next = assertDoesNotThrow(() -> controller.decide(world, perturbed,
                first.yaw(), first.pitch(), false, route, 4, 1));
        assertEquals(ElytraFlightController.Phase.CRUISE, next.phase());
    }

    @Test void lowRouteDoesNotUseTerrainPlusSixteenReserve() {
        var world = new Terrain();
        var state = new PhysicsState(new Vector3d(0, 5, 0), new Vector3d(0, 0, .8), false);
        var waypoint = new PhysicsState(new Vector3d(0, 5, 20), new Vector3d(0, 0, .8), false);
        var route = new WaypointFlightTarget(List.of(state.position(), waypoint.position()),
                waypoint.position(), false, false, 2);
        assertEquals(21, FlightSelector.routeAltitude(world, state.position(), waypoint.position()));
        var decision = new ElytraFlightController().decide(world, state, 0, 0, false, route, 4, 1);
        assertEquals(ElytraFlightController.Phase.CRUISE, decision.phase());
        PhysicsState current = state;
        for (int tick = 0; tick < 6; tick++) {
            var result = new GlidingPhysics(decision.yaw(), decision.pitch(), decision.firework())
                    .step(world, current, .299, .6);
            assertTrue(result.position().y < 10, "No cruise-height climb for a low waypoint");
            current = FlightSafety.nextState(result);
        }
    }
}
