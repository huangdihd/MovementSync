package xin.bbtt.physics;

import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class TerminalWaypointBrakingPhysicsTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");

    private World floor() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return p.y < 64 ? STONE : super.getBlockStateAt(p);
            }
        };
    }

    @Test void terminalRouteBrakesActualPoweredVelocityInsteadOfRewardingFlyby() {
        World world = floor();
        PhysicsState initial = new PhysicsState(new Vector3d(172, 66, 8),
                new Vector3d(1.29, .888, -.622), false, 0, PlayerPose.GLIDING);
        Vector3d target = new Vector3d(180, 64, 8);
        // A broad, loaded final neighbourhood isolates the steering objective
        // from corridor rejection; travel still uses real collision physics.
        var landing = new WaypointFlightTarget(List.of(target), target, true, false, 64);
        var cruise = new WaypointFlightTarget(List.of(target), target, false, false, 64);
        var controller = new ElytraFlightController();
        var stop = controller.decide(world, initial, -90, -30, true, landing, 64, 2);
        var pass = controller.decide(world, initial, -90, -30, true, cruise, 64, 2);
        PhysicsState stopped = advance(world, initial, stop);
        PhysicsState passed = advance(world, initial, pass);
        Vector3d stopVelocity = new Vector3d(target).sub(stopped.position()).div(24);
        Vector3d passVelocity = new Vector3d(target).sub(passed.position()).div(24);
        assertTrue(stopped.velocity().distance(stopVelocity) < passed.velocity().distance(passVelocity),
                "terminal control must reduce closing-velocity error relative to cruise");
        assertFalse(stop.firework());
        assertEquals(ElytraFlightController.Phase.LANDING, stop.phase());
    }

    @Test void terminalApproachLandsWithClosedLoopPhysics() {
        World world = floor();
        PhysicsState state = new PhysicsState(new Vector3d(172, 66, 8),
                new Vector3d(1.29, .888, -.622), false, 0, PlayerPose.GLIDING);
        Vector3d target = new Vector3d(180, 64, 8);
        var route = new WaypointFlightTarget(List.of(target), target, true, false, 64);
        var controller = new ElytraFlightController();
        float yaw = -90, pitch = -30;
        for (int tick = 0; tick < 120 && !state.onGround(); tick++) {
            boolean attached = tick < 20;
            var decision = controller.decide(world, state, yaw, pitch, attached, route, 64, 2);
            assertFalse(decision.firework(), "terminal approach must not renew thrust");
            yaw = decision.yaw();
            pitch = decision.pitch();
            PhysicsResult result = new GlidingPhysics(yaw, pitch, attached,
                    PlayerInput.NONE, MovementParameters.DEFAULT)
                    .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            assertFalse(result.collidedX() || result.collidedZ());
            state = FlightSafety.nextState(result);
        }
        assertTrue(state.onGround(), "closed-loop terminal approach must reach ground");
        assertTrue(state.position().distance(target) <= 2, "touchdown must remain inside arrival radius");
    }

    private PhysicsState advance(World world, PhysicsState state, ElytraFlightController.Decision decision) {
        for (int tick = 0; tick < 6; tick++) {
            PhysicsResult result = new GlidingPhysics(decision.yaw(), decision.pitch(), true,
                    PlayerInput.NONE, MovementParameters.DEFAULT)
                    .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            assertFalse(result.collidedX() || result.collidedZ());
            state = FlightSafety.nextState(result);
            if (state.onGround()) break;
        }
        return state;
    }
}
