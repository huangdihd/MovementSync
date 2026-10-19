package xin.bbtt.flight;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class P5eDangerousDescentTest extends ElytraCommandTestSupport {
    @Test void unsupportedGoalCannotAdmitDamagingAirborneArrivalDive() {
        World sea = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return p.y < 31
                        ? new BlockState("lava", 1, Map.of(), "fluid", 0, false, "STONE")
                        : super.getBlockStateAt(p);
            }
        };
        var state = new PhysicsState(new Vector3d(500, 60, 0), new Vector3d(.8, -.3, 0),
                false, 1, PlayerPose.GLIDING);
        var dive = new ElytraFlightController.Decision(-90, 85, false,
                ElytraFlightController.Phase.LANDING);
        var result = LocalGotoController.replay(sea, MovementParameters.DEFAULT, state, dive,
                false, 40, new Vector3d(516, 40, 0), true);
        assertFalse(result.steps().isEmpty());
        assertTrue(result.steps().size() < 40, "unsafe dive must not qualify as a full arrival candidate");
        assertTrue(result.end().fallDistance() < MovementParameters.DEFAULT.safeFallDistance(),
                "airborne descent must remain recoverable, not merely avoid touchdown damage");
        assertFalse(LocalGotoController.normalLanding(sea, result.end(), new Vector3d(516, 40, 0)));
    }
}
