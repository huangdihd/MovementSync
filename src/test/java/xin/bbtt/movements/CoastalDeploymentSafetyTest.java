package xin.bbtt.movements;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import java.util.Map;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.*;

/** A successful glide preview alone cannot authorize a coastal jump. */
class CoastalDeploymentSafetyTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final BlockState LAVA = new BlockState("lava", 0, Map.of("level", "0"), "empty", 0, false, "LAVA");

    private static World shore(double edge) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.x < edge && p.y < 33) return STONE;
                return p.y < 31 ? LAVA : AIR;
            }
            @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
                return x < edge && y < 33;
            }
        };
    }

    private static PhysicsState ground(double speed) {
        return new PhysicsState(new Vector3d(245.5, 33, 20.5), new Vector3d(speed, 0, 0), true);
    }

    private void install(World world, PhysicsState state) throws Exception {
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.position.set(state.position());
        movementSync.velocity.set(state.velocity());
        movementSync.onGround.set(state.onGround());
    }

    @Test void failedStartCannotAuthorizeJumpThatDriftsOffShore() throws Exception {
        World world = shore(246);
        PhysicsState state = ground(.3);
        assertNotNull(LowAltitudeFlightPlan.deployment(world, state, MovementParameters.DEFAULT, -90, 0),
                "old success-only preview flies clear of the coast");
        var abort = DeploymentSafety.assess(world, state, MovementParameters.DEFAULT, -90, 0);
        assertFalse(abort.safe());
        assertTrue(abort.reason().contains("no-deploy"));
        install(world, state);
        var lifecycle = new VanillaGlideLifecycle();
        lifecycle.allowLowAltitudeTakeoff(-90, 0);
        lifecycle.init(movementSync);
        assertEquals(state.velocity(), movementSync.velocity.get());
        var jump = MovementSync.class.getDeclaredField("jumpRequested");
        jump.setAccessible(true);
        assertFalse(((java.util.concurrent.atomic.AtomicBoolean) jump.get(movementSync)).get(),
                "unsafe launch must be rejected before queuing jump input");
        boolean[] failed = {false};
        lifecycle.tickTakeoff(movementSync, () -> failed[0] = true);
        assertTrue(failed[0]);
        assertFalse(movementSync.isGliding());
    }

    @Test void interruptedDeploymentNeedsSupportEvenWhenStationaryJumpReturns() {
        // A narrow support permits the no-deploy jump but not forward drift after deployment.
        var abort = DeploymentSafety.assess(shore(245.21), ground(0), MovementParameters.DEFAULT, -90, 0);
        assertFalse(abort.safe());
        assertTrue(abort.reason().contains("interrupted glide"), abort.reason());
    }

    @Test void broadSupportAllowsFailedStartAndEveryEarlyInterruptedBranch() {
        assertTrue(DeploymentSafety.assess(shore(260), ground(0), MovementParameters.DEFAULT, -90, 0).safe());
    }

    @Test void workerReportsAbortFailureForBothCoastalGoalLengths() throws Exception {
        for (double distance : new double[]{180, 77840 - 245}) {
            var future = LowAltitudeFlightPlan.submit(shore(246), ground(.3),
                    new Vector3d(245.5 + distance, 33, 20.5), MovementParameters.DEFAULT, false);
            try {
                assertNull(future.get(10, TimeUnit.SECONDS));
                assertEquals(LowAltitudeFlightPlan.Status.DEPLOYMENT_REJECTED, future.status());
                assertTrue(future.detail().contains("no-deploy"));
            } finally { future.cancel(true); }
        }
    }

    @Test void serverDenialDoesNotLeaveLifecyclePermanentlyAirborne() throws Exception {
        install(shore(260), new PhysicsState(new Vector3d(245.5, 34, 20.5), new Vector3d(0, -.1, 0), false));
        var lifecycle = new VanillaGlideLifecycle();
        lifecycle.init(movementSync);
        lifecycle.tickTakeoff(movementSync, () -> fail("valid start rejected"));
        assertTrue(movementSync.isGliding());
        movementSync.setGliding(false); // Authoritative server denial/correction, not a start acknowledgement.
        assertTrue(lifecycle.awaitingTakeoff(movementSync));
    }

    @Test void interruptedProofFailsClosedWithoutClearingThreadInterrupt() {
        Thread.currentThread().interrupt();
        try {
            assertFalse(DeploymentSafety.assess(shore(260), ground(0), MovementParameters.DEFAULT, -90, 0).safe());
            assertTrue(Thread.currentThread().isInterrupted());
        } finally { Thread.interrupted(); }
    }
}
