package xin.bbtt.flight;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Request-path isolation, not a reproduction of the unproven Grim Simulation alert. */
class LavaEscapeIsolationRegressionTest extends ElytraCommandTestSupport {
    private static World sea(boolean lava) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return Y > 128; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (lava && p.y < 31)
                    return new BlockState("lava", 1, Map.of(), "fluid", 0, false, "STONE");
                return super.getBlockStateAt(p);
            }
        };
    }

    private LocalGotoController.Result request(boolean lava, boolean tracking) throws Exception {
        var worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, sea(lava));
        movementSync.position.set(new Vector3d(376.4, 45, 12.7));
        movementSync.velocity.set(new Vector3d(.379, -.081, -.013));
        movementSync.onGround.set(false);
        Object owner = new Object();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var source = FlightControlAdmission.snapshot(movementSync);
        var goal = new Vector3d(376.4, 45, 12.7);
        var route = new WaypointFlightTarget(List.of(source.state().position(), goal), goal, false);
        int[] rockets = new int[64];
        Arrays.fill(rockets, 1);
        var resources = new HazardRecovery.Context(rockets, 0, false, movementSync.getLandingMemory());
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, goal, true,
                    () -> {}, tracking ? new LocalLookaheadSolver.MovingTarget(goal, new Vector3d()) : null,
                    resources, true, true));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            var result = controller.latest();
            assertNotNull(result);
            assertEquals(LocalGotoController.Status.SAFE_PREFIX, result.status());
            assertTrue(result.prefixTicks() > 0, "rejection must not masquerade as escape isolation");
            return result;
        }
    }

    @Test void trackingOverLavaDoesNotForceStaticEscapeBoost() throws Exception {
        var staticLava = request(true, false);
        assertNull(staticLava.powered(), "same-height arrival must skip ordinary powered solving");
        assertTrue(staticLava.rocket(), "positive control must activate static lava escape");
        var following = request(true, true);
        assertNotNull(following.powered(), "tracking must still evaluate ordinary powered pursuit");
        assertFalse(following.rocket(), "tracking must not inherit static lava escape thrust");
    }

    @Test void identicalSlowStaticRequestWithoutLavaDoesNotForceEscapeBoost() throws Exception {
        var staticLava = request(true, false);
        assertNull(staticLava.powered(), "same-height arrival must skip ordinary powered solving");
        assertTrue(staticLava.rocket(), "positive control must activate static lava escape");
        var nonlava = request(false, false);
        assertNull(nonlava.powered());
        assertFalse(nonlava.rocket(), "slow speed alone must not activate lava escape thrust");
    }
}
