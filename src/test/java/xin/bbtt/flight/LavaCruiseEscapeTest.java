package xin.bbtt.flight;

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

/** Field regression: low, .38 block/tick glide must not sink with 64 unused rockets. */
class LavaCruiseEscapeTest extends ElytraCommandTestSupport {
    private static World sea(double roof) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return Y > roof; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y < 31) return new BlockState("lava", 1, Map.of(), "fluid", 0, false, "STONE");
                return super.getBlockStateAt(p);
            }
        };
    }
    private static PhysicsState fieldState(double y, double speed) {
        return new PhysicsState(new Vector3d(376.4, y, 12.7),
                new Vector3d(speed, -.081, -.013), false, 1, PlayerPose.GLIDING);
    }

    @Test void cruiseMarginIncludesBothLowAltitudeAndStall() {
        assertTrue(LocalGotoController.needsLavaMargin(sea(128), fieldState(32.46, .379)));
        assertTrue(LocalGotoController.needsLavaMargin(sea(128), fieldState(36, 1.5)));
        assertTrue(LocalGotoController.needsLavaMargin(sea(128), fieldState(45, .379)));
        assertFalse(LocalGotoController.needsLavaMargin(sea(128), fieldState(45, 1.5)));
    }

    @Test void poweredEscapeGainsHeightButRejectsRoofAndUnknownTerrain() {
        var state = fieldState(32.46, .379);
        var goal = new Vector3d(229.8, 33.88, 20.75);
        var escape = LocalGotoController.poweredEscape(sea(128), MovementParameters.DEFAULT, state, 90, goal);
        assertFalse(escape.isEmpty());
        assertTrue(escape.get(0).control().firework());
        var powered = LocalGotoController.replay(sea(128), MovementParameters.DEFAULT, state,
                escape.get(0).control(), true, 12, goal, false);
        assertTrue(powered.end().position().y > state.position().y + 2,
                () -> "escape=" + escape.size() + " control=" + escape.get(0).control() + " end=" + powered.end());
        assertEquals(escape.size(), LocalGotoController.replay(sea(128), MovementParameters.DEFAULT,
                state, escape.get(0).control(), false, escape.size(), goal, false).steps().size());
        assertTrue(LocalGotoController.poweredEscape(sea(32.5), MovementParameters.DEFAULT,
                state, 90, goal).isEmpty());
        assertTrue(LocalGotoController.poweredEscape(new World() {}, MovementParameters.DEFAULT,
                state, 90, goal).isEmpty());
    }

    @Test void lowStalledReturnRequestsAvailableRocketInsteadOfUnpoweredArrival() throws Exception {
        var world = sea(128);
        var state = fieldState(32.46, .379);
        var worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.position.set(state.position());
        movementSync.velocity.set(state.velocity());
        movementSync.onGround.set(false);
        Object owner = new Object();
        movementSync.claimFlightControlOwner(owner, FlightControlAdmission.Mode.LOCAL_SOLVER);
        var source = FlightControlAdmission.snapshot(movementSync);
        // Nearby same-height route used to suppress rockets in the arrival branch.
        var goal = new Vector3d(356.4, 32.46, 12.7);
        var route = new WaypointFlightTarget(List.of(state.position(), goal), goal, false);
        int[] rockets = new int[64];
        java.util.Arrays.fill(rockets, 1);
        var resources = new HazardRecovery.Context(rockets, 0, false, movementSync.getLandingMemory());
        try (var controller = new LocalGotoController()) {
            assertTrue(controller.request(movementSync, owner, source, route, goal, true,
                    () -> {}, null, resources, true, true));
            controller.awaitIdle(5, TimeUnit.SECONDS);
            assertNotNull(controller.latest());
            assertTrue(controller.latest().rocket(), "available thrust must rescue a slow low lava glide");
            assertTrue(controller.latest().prefixTicks() > 0);
        }
    }
}
