package xin.bbtt.flight;

import java.util.List;
import java.util.concurrent.TimeUnit;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Follow uses the real bounded route worker, never the goto arrival candidate set. */
class LocalFollowControllerTest extends ElytraCommandTestSupport {
    @Test void nearbyTrackingEndpointProducesAirborneCruisePrefix() throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        });
        movementSync.position.set(new Vector3d(0, 100, 0));
        movementSync.velocity.set(new Vector3d(0, 0, .5));
        movementSync.onGround.set(false);
        Object owner = new Object();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var source = FlightControlAdmission.snapshot(movementSync);
        var goal = new Vector3d(0, 100, 8);
        var route = new WaypointFlightTarget(List.of(source.state().position(), goal), goal, true);
        var observation = new LocalLookaheadSolver.MovingTarget(goal, new Vector3d());
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, goal, false, null, observation));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            var result = controller.latest();
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, result.status());
            assertTrue(result.prefixTicks() > 0 && result.prefixTicks() <= 12);
            assertFalse(result.rocket());
            assertNotNull(result.unpowered().decision());
            assertEquals(ElytraFlightController.Phase.CRUISE, result.unpowered().decision().phase());
            assertTrue(result.unpowered().prefix().stream().noneMatch(step -> step.expected().onGround()));
        }
    }
}
