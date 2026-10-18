package xin.bbtt.movements;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class GroundedFlightStopTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final BlockState LAVA = new BlockState("lava", 0, Map.of("level", "0"), "empty", 0, false, "LAVA");

    private World installTouchdown() throws Exception {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.x < 246 && p.y < 33) return STONE;
                return p.y < 31 ? LAVA : AIR;
            }
            @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
                return x < 246 && y < 33;
            }
        };
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.position.set(new Vector3d(246.3, 33, 20.43));
        movementSync.velocity.set(new Vector3d(.08119, 0, -.007769));
        movementSync.onGround.set(true);
        movementSync.yaw.set(-95.4657f);
        return world;
    }

    private PhysicsState state() {
        return new PhysicsState(movementSync.position.get(), movementSync.velocity.get(), movementSync.onGround.get());
    }

    @Test void numericTouchdownIsNotCompletionAndNeutralCoastIsUnsafe() throws Exception {
        World world = installTouchdown();
        movementSync.setGliding(false);
        assertFalse(GroundedFlightStop.stopped(movementSync));
        assertFalse(DeploymentSafety.assessWalking(world, state(), MovementParameters.DEFAULT,
                -95.4657f, PlayerInput.NONE).safe());
    }

    @Test void releasedPlanningHoldRemainsPhysicalAndStopsBeforeLava() throws Exception {
        World world = installTouchdown();
        movementSync.setGliding(false);
        GroundedFlightStop.hold(movementSync);
        var held = movementSync.getInput();
        GroundedFlightStop.release(movementSync, held);
        assertTrue(movementSync.snapshotInput().sneak(), "cancel/finish must retain actual input, not just intent");
        PhysicsState state = state();
        for (int tick = 0; tick < 40; tick++) {
            PhysicsResult next = new WalkingPhysics(movementSync.snapshotInput(), -95.4657f)
                    .step(world, state, PlayerPose.HALF_WIDTH, (double) 1.8f);
            assertTrue(FlightHazards.safeLanding(world, next.position()));
            assertTrue(next.position().y >= 33, "retained crouch must not coast into lava");
            state = new PhysicsState(next.position(), next.velocity(), next.onGround(), next.fallDistance(), next.pose());
        }
        assertEquals(0, state.velocity().x);
        assertEquals(0, state.velocity().z);
        movementSync.position.set(state.position());
        movementSync.velocity.set(state.velocity());
        movementSync.onGround.set(state.onGround());
        assertTrue(GroundedFlightStop.stopped(movementSync));
    }

    @Test void groundedGlidingCannotBeCertifiedBySneakOrZeroVelocity() throws Exception {
        installTouchdown();
        movementSync.velocity.set(new Vector3d());
        movementSync.setGliding(true);
        GroundedFlightStop.hold(movementSync);
        assertTrue(movementSync.snapshotInput().sneak());
        assertFalse(GroundedFlightStop.stopped(movementSync));
        assertTrue(movementSync.isGliding(), "never fabricate authoritative metadata clearance");
    }

    @Test void exactEdgeGlideIsReportedUnprovedWithoutFabricatingLanding() throws Exception {
        installTouchdown();
        movementSync.setGliding(true);
        var beforePosition = new Vector3d(movementSync.position.get());
        var beforeVelocity = new Vector3d(movementSync.velocity.get());
        assertFalse(GroundedFlightStop.steerGroundedGlide(movementSync),
                "residual outward motion already exceeds the remaining support: do not promise a stop");
        assertEquals(beforePosition, movementSync.position.get());
        assertEquals(beforeVelocity, movementSync.velocity.get());
        assertTrue(movementSync.isGliding());
        assertFalse(GroundedFlightStop.stopped(movementSync));
        assertTrue(movementSync.yaw.get() > 80 && movementSync.yaw.get() < 100);
    }

    @Test void supportedGlideBrakesUntilRealMetadataAllowsWalkingStop() throws Exception {
        World world = installTouchdown();
        movementSync.position.set(new Vector3d(245.9, 33, 20.43));
        movementSync.setGliding(true);
        for (int tick = 0; tick < 40; tick++) {
            assertTrue(GroundedFlightStop.steerGroundedGlide(movementSync));
            var next = new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(), false,
                    PlayerInput.NONE, MovementParameters.DEFAULT)
                    .step(world, state(), PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            assertTrue(next.onGround());
            assertTrue(next.position().x < 246.3);
            assertTrue(FlightHazards.safeLanding(world, next.position()));
            movementSync.position.set(next.position());
            movementSync.velocity.set(next.velocity());
            movementSync.onGround.set(next.onGround());
            assertFalse(GroundedFlightStop.stopped(movementSync));
        }
        movementSync.setGliding(false); // Simulated authoritative metadata, not the controller.
        for (int tick = 0; tick < 40; tick++) {
            GroundedFlightStop.hold(movementSync);
            var next = new WalkingPhysics(movementSync.snapshotInput(), movementSync.yaw.get())
                    .step(world, state(), PlayerPose.HALF_WIDTH, (double) 1.8f);
            assertTrue(next.onGround());
            assertTrue(FlightHazards.safeLanding(world, next.position()));
            movementSync.position.set(next.position());
            movementSync.velocity.set(next.velocity());
            movementSync.onGround.set(next.onGround());
        }
        assertTrue(GroundedFlightStop.stopped(movementSync));
    }

    @Test void oldPlanningOwnerCannotEraseNextWalkingCommand() throws Exception {
        installTouchdown();
        GroundedFlightStop.hold(movementSync);
        var owned = movementSync.getInput();
        var walk = new PlayerInput(true, false, false, false, false, false, false);
        movementSync.setInput(walk);
        GroundedFlightStop.release(movementSync, owned);
        assertSame(walk, movementSync.getInput());
    }
}
