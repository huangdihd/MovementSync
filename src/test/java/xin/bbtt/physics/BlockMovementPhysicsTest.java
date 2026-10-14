package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.world.World;

import java.util.OptionalDouble;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

final class BlockMovementPhysicsTest {
    @Test
    void settingsPreserveOrdinaryGroundAndVanillaFloatOverrides() {
        assertEquals(0.6, properties("stone").slipperiness());
        assertEquals((double) 0.98f, properties("ice").slipperiness());
        assertEquals((double) 0.989f, properties("blue_ice").slipperiness());
        assertEquals((double) 0.8f, properties("slime_block").slipperiness());
        assertEquals((double) 0.66f, properties("red_bed").bounce());
    }

    @Test
    void slipperinessControlsAccelerationAndDrag() {
        World world = new BlockWorld("ice", "air");
        PlayerInput input = new PlayerInput(true, false, false, false, false, false, false);
        PhysicsResult result = new WalkingPhysics(input, 0).step(world,
                new PhysicsState(new Vector3d(0.5, 1, 0.5), new Vector3d(), true), 0.3, 1.8);
        double friction = (double) 0.98f;
        double acceleration = 0.98 * MovementParameters.DEFAULT.movementSpeed(false)
                * 0.21600002 / (friction * friction * friction);
        assertEquals(0.5 + acceleration, result.position().z, 1e-12);
        assertEquals(acceleration * friction * 0.91, result.velocity().z, 1e-12);
    }

    @Test
    void honeyJumpAndSoulSandVelocityUseSupportProperties() {
        PlayerInput jump = new PlayerInput(false, false, false, false, true, false, false);
        PhysicsResult result = new WalkingPhysics(jump, 0).step(new BlockWorld("honey_block", "air"),
                new PhysicsState(new Vector3d(0.5, 1, 0.5), new Vector3d(), true), 0.3, 1.8);
        assertEquals(1 + xin.bbtt.MovementSync.jumpVelocity * 0.5, result.position().y, 1e-7);
        result = new WalkingPhysics().step(new BlockWorld("soul_sand", "air"),
                new PhysicsState(new Vector3d(0.5, 1, 0.5), new Vector3d(0.1, 0, 0), true), 0.3, 1.8);
        assertEquals(0.1 * (double) 0.4f * 0.6 * 0.91, result.velocity().x, 1e-12);
    }

    @Test
    void slimeAndBedBounceAndSneakingCancelsBoth() {
        for (String block : new String[]{"slime_block", "red_bed"}) {
            PhysicsState start = new PhysicsState(new Vector3d(0.5, 1.1, 0.5), new Vector3d(0, -0.4, 0), false);
            PhysicsResult result = new WalkingPhysics().step(new BlockWorld(block, "air"), start, 0.3, 1.8);
            assertEquals(1, result.position().y, 1e-12);
            assertEquals((0.4 * properties(block).bounce() - 0.08) * xin.bbtt.MovementSync.verticalDrag,
                    result.velocity().y, 1e-12);
            PlayerInput sneak = new PlayerInput(false, false, false, false, false, true, false);
            result = new WalkingPhysics(sneak, 0).step(new BlockWorld(block, "air"), start, 0.3, 1.8);
            assertEquals(-0.08 * xin.bbtt.MovementSync.verticalDrag, result.velocity().y, 1e-12);
        }
    }

    @Test
    void stuckBlocksScaleDisplacementThenResetMomentum() {
        for (String block : new String[]{"cobweb", "sweet_berry_bush", "powder_snow"}) {
            PhysicsResult result = new WalkingPhysics().step(new BlockWorld("air", block),
                    new PhysicsState(new Vector3d(0.5, 2, 0.5), new Vector3d(0.1, -0.2, 0.1), false), 0.3, 1.8);
            assertEquals(0.5 + 0.1 * properties(block).stuckHorizontal(), result.position().x, 1e-12);
            assertEquals(2 - 0.2 * properties(block).stuckVertical(), result.position().y, 1e-12);
            assertEquals(0, result.velocity().x);
            assertEquals(-0.08 * xin.bbtt.MovementSync.verticalDrag, result.velocity().y, 1e-12);
        }
    }

    @Test
    void honeySlideReversesTravelGravityBeforeApplyingSideDrag() {
        Vector3d velocity = new Vector3d(0.4, (-0.5 - 0.08) * (double) 0.98f, 0.2);
        EnvironmentalPhysics.applyHoneySlide(velocity);
        assertEquals(0.04, velocity.x, 1e-12);
        assertEquals(0.02, velocity.z, 1e-12);
        assertEquals(-0.13 * (double) 0.98f, velocity.y, 1e-12);
        Vector3d rising = new Vector3d(1, 0.2, 1);
        EnvironmentalPhysics.applyHoneySlide(rising);
        assertEquals(new Vector3d(1, 0.2, 1), rising);
    }

    @Test
    void bubblesUseSeparateSurfaceAndSubmergedCaps() {
        assertEquals(0.06, EnvironmentalPhysics.bubbleVelocity(0, false, false));
        assertEquals(0.1, EnvironmentalPhysics.bubbleVelocity(0, false, true));
        assertEquals(0.7, EnvironmentalPhysics.bubbleVelocity(0.69, false, false));
        assertEquals(1.8, EnvironmentalPhysics.bubbleVelocity(1.79, false, true));
        assertEquals(-0.3, EnvironmentalPhysics.bubbleVelocity(-0.29, true, false));
        assertEquals(-0.9, EnvironmentalPhysics.bubbleVelocity(-0.89, true, true));
    }

    @Test
    void honeyRequiresSideContactAndPowderSnowRequiresFeetContact() {
        World honey = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.x >= 0 && position.x < 1 && position.y >= 1 && position.y < 2
                        ? state("honey_block") : state("air");
            }
        };
        Vector3d velocity = new Vector3d(0.4, (-0.5 - 0.08) * (double) 0.98f, 0);
        EnvironmentalPhysics.applyContactVelocity(honey, new Vector3d(1.24, 1.5, 0.5), velocity, 0.3, 1.8, false);
        assertEquals(0.04, velocity.x, 1e-12);
        Vector3d grounded = new Vector3d(0.4, -0.5, 0);
        EnvironmentalPhysics.applyContactVelocity(honey, new Vector3d(1.24, 1.5, 0.5), grounded, 0.3, 1.8, true);
        assertEquals(-0.5, grounded.y);
        World snow = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.x < 1 ? state("powder_snow") : state("air");
            }
        };
        assertEquals(new Vector3d(1), EnvironmentalPhysics.stuckMultiplier(snow,
                new Vector3d(1.2, 1, 0.5), 0.3, 1.8));
        assertEquals((double) 0.9f, EnvironmentalPhysics.stuckMultiplier(snow,
                new Vector3d(0.8, 1, 0.5), 0.3, 1.8).x);
    }

    private static BlockMovementProperties properties(String name) {
        return BlockMovementProperties.of(state(name));
    }

    private static BlockState state(String name) {
        for (int id = 0; id < BlockStateParser.getBlockStateRegistrySize(); id++) {
            BlockState state = BlockStateParser.Instance.parseStateId(id);
            if (state.blockName().equals(name)) return state;
        }
        throw new AssertionError("Unknown block " + name);
    }

    private static final class BlockWorld extends World {
        private final BlockState support;
        private final BlockState contact;
        BlockWorld(String support, String contact) {
            this.support = state(support);
            this.contact = state(contact);
        }
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d position) {
            return position.y < 1 ? support : contact;
        }
        @Override public boolean isBoxColliding(double a, double b, double c, double d, double e, double f) {
            return false;
        }
        @Override public OptionalDouble findHighestCollisionTopExcluding(double a, double minimumY, double c,
                double d, double maximumY, double f, Set<org.joml.Vector3i> excluded) {
            return !support.blockName().equals("air") && minimumY <= 1 && maximumY >= 1
                    ? OptionalDouble.of(1) : OptionalDouble.empty();
        }
    }
}
