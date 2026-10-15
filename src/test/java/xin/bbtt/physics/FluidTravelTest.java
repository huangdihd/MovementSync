package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;
import java.util.Map;
import static org.junit.jupiter.api.Assertions.*;

final class FluidTravelTest {
    private static final double HALF_WIDTH = (double) 0.6f / 2;
    private static World uniform(String name) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return new BlockState(name, 0, Map.of("level", "0"), "empty", 0, false, "");
            }
            @Override public boolean isBoxColliding(double a, double b, double c, double d, double e, double f) { return false; }
        };
    }
    private static PhysicsResult step(World world, PhysicsState state, PlayerInput input) {
        return new WalkingPhysics(input, 0).step(world, state, HALF_WIDTH, 1.8);
    }
    private static PhysicsState next(PhysicsResult result) {
        return new PhysicsState(result.position(), result.velocity(), result.onGround());
    }
    @Test void stillWaterSinkingSequenceMatchesTravelInWater() {
        // Yarn 1.21.11 LivingEntity.travelInWater + applyFluidMovingSpeed.
        PhysicsState state = new PhysicsState(new Vector3d(0.5, 20, 0.5), new Vector3d(), false);
        double expected = 0;
        for (int tick = 0; tick < 40; tick++) {
            PhysicsResult result = step(uniform("water"), state, PlayerInput.NONE);
            assertEquals(state.position().y + expected, result.position().y, 1e-12);
            expected = expected * (double) 0.8f - 0.005;
            assertEquals(expected, result.velocity().y, 1e-12);
            state = next(result);
        }
    }
    @Test void jumpingWaterSequenceMatchesSwimUpward() {
        // LivingEntity.swimUpward adds 0.04F before travelInWater.
        PhysicsState state = new PhysicsState(new Vector3d(0.5, 20, 0.5), new Vector3d(), false);
        PlayerInput jump = new PlayerInput(false, false, false, false, true, false, false);
        double expected = 0;
        for (int tick = 0; tick < 30; tick++) {
            PhysicsResult result = step(uniform("water"), state, jump);
            assertEquals(state.position().y + expected + (double) 0.04f, result.position().y, 1e-12);
            expected = (expected + (double) 0.04f) * (double) 0.8f - 0.005;
            assertEquals(expected, result.velocity().y, 1e-12);
            state = next(result);
        }
    }
    @Test void horizontalWaterAccelerationAndSprintNormalization() {
        PhysicsState state = new PhysicsState(new Vector3d(0.5, 20, 0.5), new Vector3d(), false);
        for (boolean sprint : new boolean[]{false, true}) {
            PhysicsResult result = step(uniform("water"), state,
                    new PlayerInput(true, false, false, false, false, false, sprint));
            assertEquals(0.5 + 0.98 * (double) 0.02f, result.position().z, 1e-12);
            assertEquals(0.98 * (double) 0.02f * (double) 0.8f, result.velocity().z, 1e-12);
        }
    }
    @Test void deepLavaSinkingAndJumpingFollowTravelInLava() {
        // LivingEntity.travelInLava: deep lava halves velocity then subtracts gravity/4.
        for (boolean jump : new boolean[]{false, true}) {
            PhysicsState state = new PhysicsState(new Vector3d(0.5, 20, 0.5), new Vector3d(), false);
            double expected = 0;
            for (int tick = 0; tick < 20; tick++) {
                if (Math.abs(expected) < 0.003) expected = 0;
                double movement = expected + (jump ? (double) 0.04f : 0);
                PhysicsResult result = step(uniform("lava"), state,
                        new PlayerInput(true, false, false, false, jump, false, false));
                assertEquals(state.position().y + movement, result.position().y, 1e-12);
                expected = movement * 0.5 - 0.02;
                assertEquals(expected, result.velocity().y, 1e-12);
                state = next(result);
            }
        }
    }
    @Test void mediumIsSelectedBeforeMovementOnEntryAndExit() {
        // Entity.updateWaterState precedes LivingEntity.travel; crossing takes effect next tick.
        World waterBelow = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return new BlockState(position.y < 0 ? "water" : "air", 0,
                        Map.of("level", "0"), "empty", 0, false, "");
            }
            @Override public boolean isBoxColliding(double a, double b, double c, double d, double e, double f) { return false; }
        };
        PhysicsResult entry = step(waterBelow,
                new PhysicsState(new Vector3d(0.5, 0.2, 0.5), new Vector3d(0, -0.5, 0), false), PlayerInput.NONE);
        assertEquals(-0.3, entry.position().y, 1e-12);
        assertEquals((-0.5 - 0.08) * (double) 0.98f, entry.velocity().y, 1e-12);
        PhysicsResult underwater = step(waterBelow, next(entry), PlayerInput.NONE);
        assertEquals(entry.velocity().y * (double) 0.8f - 0.005, underwater.velocity().y, 1e-12);
        PhysicsResult exit = step(waterBelow,
                new PhysicsState(new Vector3d(0.5, -0.2, 0.5), new Vector3d(0, 0.4, 0), false), PlayerInput.NONE);
        assertEquals(0.2, exit.position().y, 1e-12);
        assertEquals(0.4 * (double) 0.8f - 0.005, exit.velocity().y, 1e-12);
        PhysicsResult airborne = step(waterBelow, next(exit), PlayerInput.NONE);
        assertEquals((exit.velocity().y - 0.08) * (double) 0.98f, airborne.velocity().y, 1e-12);
    }

    @Test void ladderClampsFallingAndSneakingAndJumping() {
        // LivingEntity.applyClimbingSpeed then applyMovementInput then travelMidAir.
        PhysicsState state = new PhysicsState(new Vector3d(0.5, 20, 0.5), new Vector3d(0, -0.8, 0), false);
        PhysicsResult falling = step(uniform("ladder"), state, PlayerInput.NONE);
        assertEquals(20 - (double) 0.15f, falling.position().y, 1e-12);
        PhysicsResult sneak = step(uniform("ladder"), state,
                new PlayerInput(false, false, false, false, false, true, false));
        assertEquals(20, sneak.position().y);
        PhysicsResult jump = step(uniform("ladder"), state,
                new PlayerInput(false, false, false, false, true, false, false));
        assertEquals((0.2 - 0.08) * (double) 0.98f, jump.velocity().y, 1e-12);
    }
}
