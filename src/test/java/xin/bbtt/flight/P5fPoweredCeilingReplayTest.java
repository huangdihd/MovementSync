package xin.bbtt.flight;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class P5fPoweredCeilingReplayTest {
    private World roof() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return Y > 12; }
        };
    }

    @Test void attachedThrustCannotMaskTravelCeilingContact() {
        var world = roof();
        var state = new PhysicsState(new Vector3d(.5, 12 - PlayerPose.GLIDING.height(), .5),
                new Vector3d(0, .4, .3), false, 0, PlayerPose.GLIDING);
        var control = new ElytraFlightController.Decision(0, -20, false, ElytraFlightController.Phase.CRUISE);
        var travel = new GlidingPhysics(0, -20, false, PlayerInput.NONE, MovementParameters.DEFAULT)
                .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        var boosted = new GlidingPhysics(0, -20, true, PlayerInput.NONE, MovementParameters.DEFAULT)
                .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        assertEquals(0, travel.velocity().y, "fixture must actually clip upward travel");
        assertTrue(boosted.velocity().y > 0, "post-travel thrust masks zero vertical velocity");
        assertTrue(LocalGotoController.replay(world, MovementParameters.DEFAULT, state, control,
                true, 12, new Vector3d(0, 20, 20), false).steps().isEmpty(),
                "reject the first ceiling contact, not just a later collision");
        assertTrue(LocalGotoController.poweredEscape(world, MovementParameters.DEFAULT, state, 0,
                new Vector3d(0, 20, 20)).isEmpty(), "attached escape must not authorize an above-roof climb");
    }
}
