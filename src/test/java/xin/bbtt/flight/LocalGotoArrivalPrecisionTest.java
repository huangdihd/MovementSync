package xin.bbtt.flight;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

class LocalGotoArrivalPrecisionTest extends ElytraCommandTestSupport {
    @Test void lateRecedingArrivalStillLandsOnTheSupportedGoal() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 64 || position.y >= 80
                        ? new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE")
                        : super.getBlockStateAt(position);
            }
        };
        var goal = new Vector3d(16.5, 64, 608.5);
        var state = new PhysicsState(new Vector3d(17.92, 64.64, 608.5),
                new Vector3d(.03115, -.02154, .4408), false, 1.02154, PlayerPose.GLIDING);
        float yaw = 163.54097f;
        for (int tick = 0; tick < 40 && !state.onGround(); tick++) {
            var controls = LocalGotoController.arrival(world, MovementParameters.DEFAULT,
                    state, goal, false, yaw);
            assertFalse(controls.isEmpty(), "arrival must retain a proved continuation at " + state);
            var control = controls.get(0).control();
            var next = new GlidingPhysics(control.yaw(), control.pitch(), false,
                    PlayerInput.NONE, MovementParameters.DEFAULT)
                    .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            assertTrue(LocalAirRoute.visible(world, state.position(), next.position(), PlayerPose.GLIDING));
            assertFalse(next.collidedX() || next.collidedZ());
            assertTrue(FlightHazards.safeTravel(world, state, next));
            state = new PhysicsState(next.position(), next.velocity(), next.onGround(),
                    next.fallDistance(), PlayerPose.GLIDING);
            yaw = control.yaw();
        }
        assertTrue(state.onGround(), "arrival must physically reach support");
        // This late seed is already receding from the goal 0.64 blocks above support; bounded
        // constant, two-stage and beam searches found no landing within 1.5 blocks (best 1.65,
        // movementsync-migration/builds/terminal-precision-private/EVIDENCE.md). Terminal
        // precision is recovered one tick earlier, see AttachedArrivalPrecisionTest; here
        // replanning must still close on the ordinary supported-goal landing contract.
        assertTrue(LocalGotoController.normalLanding(world, state, goal),
                "receding-horizon replanning must still land on the supported goal: " + state);
    }
}
