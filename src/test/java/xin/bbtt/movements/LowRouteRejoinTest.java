package xin.bbtt.movements;

import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.flight.FlightContext;
import xin.bbtt.flight.FlightContextBuilder;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class LowRouteRejoinTest {
    private WaypointFlightTarget route() {
        return new WaypointFlightTarget(List.of(new Vector3d(0, 20, 0), new Vector3d(0, 20, 80)),
                new Vector3d(0, 20, 80), false);
    }

    @Test void projectionAimsForwardRatherThanBackAtRouteStart() {
        var projection = ElytraFlyMovement.forwardRouteProjection(route(), new Vector3d(4, 20, 40));
        assertEquals(new Vector3d(0, 20, 40), projection.point());
        assertEquals(new Vector3d(0, 0, 1), projection.direction());
        assertEquals(new Vector3d(0, 20, 46), projection.aim());
    }

    @Test void degenerateRouteDoesNotInventDirection() {
        var single = new WaypointFlightTarget(List.of(new Vector3d()), new Vector3d(), false);
        assertNull(ElytraFlyMovement.forwardRouteProjection(single, new Vector3d()));
    }

    private FlightContext context(World world) {
        return new FlightContext(world,
                new PhysicsState(new Vector3d(4, 20, 20), new Vector3d(0, 0, .4), false),
                MovementParameters.DEFAULT, DamageParameters.NONE, 20,
                new Vector3d(0, 20, 80), 180, 60, 0, false, 2, false, Map.of());
    }

    @Test void staleBackwardsControlsAreReplacedOnlyBySweptForwardProgress() {
        var world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return y < 0; }
        };
        var context = context(world);
        var config = new FlightContextBuilder().addStandard().build();
        var recovery = new HazardRecovery.Context(new int[0], 0, false, null);
        var decision = ElytraFlyMovement.lowRouteRejoin(context, route(), config, recovery);
        assertNotNull(decision);
        var end = ElytraFlyMovement.airbornePursuitPreview(context, decision, config);
        assertNotNull(end);
        assertTrue(end.position().z >= context.state().position().z);
        var aim = ElytraFlyMovement.forwardRouteProjection(route(), context.state().position()).aim();
        assertTrue(end.position().distanceSquared(aim) < context.state().position().distanceSquared(aim));
        assertTrue(ElytraFlyMovement.lowDecisionSafe(context, decision, config, recovery));
    }

    @Test void geometricRouteCannotProveUnknownTerrain() {
        var world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
        };
        assertNull(ElytraFlyMovement.lowRouteRejoin(context(world), route(),
                new FlightContextBuilder().addStandard().build(),
                new HazardRecovery.Context(new int[0], 0, false, null)));
    }
}
