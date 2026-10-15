package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.tasks.VerticalCollisionResolver;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class PowderSnowFallTest {
    private static final double HALF_WIDTH = (double) 0.6F / 2;

    @Test
    void fallingShapeRequiresStrictThresholdAndUsesFloatHeight() {
        World world = new SnowWorld();
        Vector3d feet = new Vector3d(0.5, -58, 0.5);
        assertEquals(-58.5, VerticalCollisionResolver.resolveDownwardY(world, feet, HALF_WIDTH, -0.5, 2.5));
        assertEquals(-59 + (double) 0.9F,
                VerticalCollisionResolver.resolveDownwardY(world, feet, HALF_WIDTH, -0.5, 2.500001));
        assertEquals(-57.5, VerticalCollisionResolver.resolveDownwardY(world, feet, HALF_WIDTH, 0.5, 3));
        assertEquals(-58.5, VerticalCollisionResolver.resolveDownwardY(world,
                new Vector3d(1 + HALF_WIDTH, -58, 0.5), HALF_WIDTH, -0.5, 3));
        assertFalse(world.isOnGround(feet), "Static snow collision must remain empty");
    }

    @Test
    void fallDistanceAccumulatesActualDescentAndResetsOnLanding() {
        PhysicsState state = new PhysicsState(new Vector3d(0.5, -55, 0.5), new Vector3d(), false);
        WalkingPhysics physics = new WalkingPhysics();
        boolean landed = false;
        for (int tick = 0; tick < 30; tick++) {
            PhysicsResult result = physics.step(new SnowWorld(), state, HALF_WIDTH, 1.8);
            if (result.onGround()) {
                assertEquals(-59 + (double) 0.9F, result.position().y);
                assertEquals(0, result.fallDistance());
                PhysicsResult next = physics.step(new SnowWorld(), nextState(result), HALF_WIDTH, 1.8);
                assertTrue(next.position().y < result.position().y, "No boots: sink after the falling impact");
                assertFalse(next.onGround());
                assertEquals(0, next.fallDistance(), "Snow's stuck callback resets fall distance");
                landed = true;
                break;
            }
            assertEquals(-55 - result.position().y, result.fallDistance(), 1e-12);
            state = nextState(result);
        }
        assertTrue(landed, "A three-block fall must hit the contextual snow shape");
    }

    @Test
    void effectsResetBeforeTravelRatherThanDiscardingThisTicksDescent() {
        MovementParameters defaults = MovementParameters.DEFAULT;
        for (boolean slowFalling : new boolean[]{false, true}) {
            MovementParameters parameters = new MovementParameters(defaults.movementSpeed(),
                    defaults.sprintingMovementSpeed(), defaults.jumpStrength(), defaults.jumpBoostVelocity(),
                    defaults.gravity(), defaults.stepHeight(), defaults.sneakingSpeed(), slowFalling,
                    slowFalling ? -1 : 0, true);
            PhysicsResult result = new WalkingPhysics(PlayerInput.NONE, 0, true, parameters).step(
                    new SnowWorld(), new PhysicsState(new Vector3d(0.5, -55, 0.5),
                            new Vector3d(0, -0.25, 0), false, 10), HALF_WIDTH, 1.8);
            assertEquals(0.25, result.fallDistance(), 1e-12);
            PhysicsResult rising = new WalkingPhysics(PlayerInput.NONE, 0, true, parameters).step(
                    new SnowWorld(), new PhysicsState(new Vector3d(0.5, -55, 0.5),
                            new Vector3d(0, 0.25, 0), false, 10), HALF_WIDTH, 1.8);
            assertEquals(0, rising.fallDistance());
        }
    }

    @Test
    void existingStuckContactCannotReuseLargeFallDistance() {
        PhysicsResult result = new WalkingPhysics().step(new SnowWorld(),
                new PhysicsState(new Vector3d(0.5, -58.05, 0.5), new Vector3d(0, -0.2, 0), false, 10),
                HALF_WIDTH, 1.8);
        assertEquals(-58.35, result.position().y, 1e-12);
        assertFalse(result.onGround());
        assertEquals(0, result.fallDistance());
    }

    @Test
    void legacyConstructorsStartWithNoFallDistance() {
        assertEquals(0, new PhysicsState(new Vector3d(), new Vector3d(), false).fallDistance());
        assertEquals(0, new PhysicsResult(new Vector3d(), new Vector3d(), false, false).fallDistance());
        assertEquals(0, new PhysicsResult(new Vector3d(), new Vector3d(), false, false, true).fallDistance());
    }

    private static PhysicsState nextState(PhysicsResult result) {
        return new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance());
    }

    private static final class SnowWorld extends World {
        private final BlockState snow = block("powder_snow");
        private final BlockState air = block("air");

        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d position) {
            return Math.floor(position.x) == 0 && Math.floor(position.y) == -59 && Math.floor(position.z) == 0
                    ? snow : air;
        }

        private static BlockState block(String name) {
            for (int id = 0; id < BlockStateParser.getBlockStateRegistrySize(); id++) {
                BlockState state = BlockStateParser.Instance.parseStateId(id);
                if (state.blockName().equals(name)) return state;
            }
            throw new AssertionError("Unknown block " + name);
        }
    }
}
