package xin.bbtt.physics;

import java.lang.reflect.Method;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class HazardRecoveryFallDistancePhysicsTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private final World world = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d p) {
            return p.y < 64 ? STONE : super.getBlockStateAt(p);
        }
    };

    private PhysicsState falling(double velocityY) {
        return new PhysicsState(new Vector3d(8, 64.1, 8), new Vector3d(0, velocityY, 0),
                false, 30, PlayerPose.GLIDING);
    }

    @Test void reserveRejectsDamagingSupportEvenThoughContactClearsResultFallDistance() throws Exception {
        for (float pitch : new float[] {0, -10, -30, 10, 25}) {
            PhysicsState before = falling(-.5);
            PhysicsResult result = new GlidingPhysics(0, pitch, false, PlayerInput.NONE, MovementParameters.DEFAULT)
                    .step(world, before, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            assertTrue(result.onGround());
            assertEquals(0, result.fallDistance());
            assertTrue(new FlightSafety(MovementParameters.DEFAULT, DamageParameters.NONE, 20)
                    .collisionDamage(world, before, result, 0, pitch, false).raw() > 0);
            assertNull(rollout(before, pitch), "safe-looking support must not erase pre-contact fall damage");
        }
    }

    @Test void strictSlowDescentResetPermitsActuallyHarmlessContact() throws Exception {
        assertNotNull(rollout(falling(Math.nextUp(-.5)), 0),
                "limitFallDistance resets only above -0.5, before travel");
    }

    @Test void reservePitchesSettleAboveFallResetThresholdInDefaultPhysics() {
        World air = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        };
        for (float pitch : new float[] {0, -10, -30, 10, 25}) {
            PhysicsState state = new PhysicsState(new Vector3d(8, 10000, 8), new Vector3d(),
                    false, 0, PlayerPose.GLIDING);
            for (int tick = 0; tick < 600; tick++) {
                state = FlightSafety.nextState(new GlidingPhysics(0, pitch, false,
                        PlayerInput.NONE, MovementParameters.DEFAULT)
                        .step(air, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height()));
            }
            assertTrue(state.velocity().y > FlightSafety.FALL_RESET_SPEED, "pitch=" + pitch);
            assertTrue(state.fallDistance() < 2, "steady glide must reset accumulated fall distance");
        }
    }

    private Object rollout(PhysicsState state, float pitch) throws Exception {
        // Exercise the actual terminal proof rather than assess's intentional
        // NOT_EXPOSED shortcut for a column with ordinary dry support.
        Method method = HazardRecovery.class.getDeclaredMethod("rollout", World.class, PhysicsState.class,
                MovementParameters.class, float.class, float.class, boolean.class,
                HazardRecovery.Context.class, HazardRecovery.Budget.class);
        method.setAccessible(true);
        return method.invoke(null, world, state, MovementParameters.DEFAULT, 0f, pitch, false,
                HazardRecovery.Context.EMPTY, new HazardRecovery.Budget(600));
    }
}
