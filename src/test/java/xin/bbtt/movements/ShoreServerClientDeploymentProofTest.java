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
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.*;

/** Independent server walking/client delayed-glide branches of the recorded shore launch. */
class ShoreServerClientDeploymentProofTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final BlockState LAVA = new BlockState("lava", 0, Map.of("level", "0"), "empty", 0, false, "LAVA");

    private static World shore(int geometry) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.x < (geometry == 0 ? 247 : 246) && p.y < 33) return STONE;
                return geometry == 1 && p.y < 33 ? LAVA : AIR;
            }
            @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
                // Full-block cliff, adjacent lava, or a half-block step beyond the shore.
                return x < (geometry == 0 ? 247 : 246) && y < 33
                        || geometry == 2 && X > 246 && y < 32.5;
            }
        };
    }

    private static PhysicsState launch() {
        return new PhysicsState(new Vector3d(245.5, 33, 20.5), new Vector3d(0, -.0784, 0), true);
    }

    @Test void unsupportedShoreLaunchRejectedBeforeJump() throws Exception {
        var worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        var jumpField = MovementSync.class.getDeclaredField("jumpRequested");
        jumpField.setAccessible(true);
        // Full-column support is covered by the safe-stop tests below; round 13
        // rejected it only because positions differed, not because it was unsafe.
        for (int geometry = 1; geometry < 3; geometry++) {
            World world = shore(geometry);
            assertFalse(DeploymentSafety.assess(world, launch(), MovementParameters.DEFAULT, -95.4657f, 0).safe());
            worldField.set(movementSync, world);
            movementSync.position.set(launch().position());
            movementSync.velocity.set(launch().velocity());
            movementSync.onGround.set(true);
            movementSync.setGliding(false);
            var lifecycle = new VanillaGlideLifecycle();
            lifecycle.allowLowAltitudeTakeoff(-95.4657f, 0);
            lifecycle.init(movementSync);
            assertFalse(((AtomicBoolean) jumpField.get(movementSync)).get(), "no unsafe jump may be queued");
            boolean[] rejected = {false};
            lifecycle.tickTakeoff(movementSync, () -> rejected[0] = true);
            assertTrue(rejected[0]);
            assertFalse(movementSync.isGliding());
            var future = LowAltitudeFlightPlan.submit(world, launch(), new Vector3d(425.5, 33, 3.3),
                    MovementParameters.DEFAULT, false);
            assertNull(future.get(10, TimeUnit.SECONDS));
            assertEquals(LowAltitudeFlightPlan.Status.DEPLOYMENT_REJECTED, future.status());
        }
    }

    @Test void observedDeploymentEitherStopsOnFullColumnOrRejectsUnsupportedShore() {
        for (int geometry = 0; geometry < 3; geometry++) {
            PhysicsState state = launch();
            for (int tick = 0; tick < 6; tick++) {
                var next = new WalkingPhysics(PlayerInput.NONE.withJump(tick == 0), -95.4657f, true,
                        MovementParameters.DEFAULT).step(shore(geometry), state, PlayerPose.HALF_WIDTH, (double) 1.8f);
                state = new PhysicsState(next.position(), next.velocity(), next.onGround(), next.fallDistance(), next.pose());
            }
            assertEquals(34.2522, state.position().y, 1e-4);
            assertEquals(-.07544, state.velocity().y, 1e-5);
            for (int delay : new int[]{1, 2, 4}) {
                assertEquals(geometry == 0, DeploymentSafety.assessLanding(shore(geometry), state, MovementParameters.DEFAULT,
                        -95.4657f, 0, delay).safe(), "geometry=" + geometry + " delay=" + delay);
            }
        }
    }

    @Test void loggedContactRequiresSafeSupportRatherThanEqualPositions() {
        var contact = new PhysicsState(new Vector3d(246.3, 33, 20.42),
                new Vector3d(.06253, 0, -.005983), true, 0, PlayerPose.GLIDING);
        for (int geometry = 0; geometry < 3; geometry++) for (int delay : new int[]{1, 2, 4}) {
            assertEquals(geometry == 0, DeploymentSafety.assessLanding(shore(geometry), contact, MovementParameters.DEFAULT,
                    -95.4657f, 0, delay).safe(), "geometry=" + geometry + " metadata delay=" + delay);
        }
    }

    @Test void widePlatformLaunchAndDescendingDeploymentAreAllowed() throws Exception {
        // Synthetic broad platform, not a reconstruction of the uncaptured Nether world.
        World broad = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 36 ? STONE : AIR; }
            @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
                return y < 36;
            }
        };
        PhysicsState initial = new PhysicsState(new Vector3d(-3.5, 36, -1.5),
                new Vector3d(0, -.0784, 0), true);
        assertTrue(DeploymentSafety.assess(broad, initial, MovementParameters.DEFAULT, -99.5f, 0).safe());
        var worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, broad);
        movementSync.position.set(initial.position());
        movementSync.velocity.set(initial.velocity());
        movementSync.onGround.set(true);
        movementSync.setGliding(false);
        var lifecycle = new VanillaGlideLifecycle();
        lifecycle.allowLowAltitudeTakeoff(-99.5f, 0);
        lifecycle.init(movementSync);
        var jumpField = MovementSync.class.getDeclaredField("jumpRequested");
        jumpField.setAccessible(true);
        assertTrue(((AtomicBoolean) jumpField.get(movementSync)).get(), "safe platform queues jump");
        PhysicsState state = initial;
        for (int tick = 0; tick < 6; tick++) {
            var next = new WalkingPhysics(PlayerInput.NONE.withJump(tick == 0), -99.5f, true,
                    MovementParameters.DEFAULT).step(broad, state, PlayerPose.HALF_WIDTH, (double) 1.8f);
            state = new PhysicsState(next.position(), next.velocity(), next.onGround(), next.fallDistance(), next.pose());
            if (tick == 0) assertEquals(36.42, state.position().y, 1e-5);
        }
        assertEquals(37.2522, state.position().y, 1e-4);
        assertEquals(-.07544, state.velocity().y, 1e-5);
        assertTrue(DeploymentSafety.assess(broad, state, MovementParameters.DEFAULT, -99.5f, 0).safe());
        // Exercise the two-sided proof explicitly, even though broad inland launch
        // assessment can use its existing early-interruption branch.
        for (int delay : new int[]{1, 2, 4}) {
            var proof = DeploymentSafety.assessLanding(broad, state, MovementParameters.DEFAULT, -99.5f, 0, delay);
            assertTrue(proof.safe(), proof.reason());
        }
    }

    @Test void broadSupportAllowsSafeStopsDespiteMetadataPositionDivergence() {
        World broad = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 33 ? STONE : AIR; }
            @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
                return y < 33;
            }
        };
        var contact = new PhysicsState(new Vector3d(245.5, 33, 20.5),
                new Vector3d(0, 0, 0), true, 0, PlayerPose.GLIDING);
        for (int delay : new int[]{1, 2, 4}) {
            var proof = DeploymentSafety.assessLanding(broad, contact, MovementParameters.DEFAULT, -95.4657f, 0, delay);
            // Round 14: replace round 13's equality veto with independent safe stops.
            // Pitch-zero glide generates horizontal lift even from rest, which is allowed.
            assertTrue(proof.safe(), proof.reason());
        }
    }
}
