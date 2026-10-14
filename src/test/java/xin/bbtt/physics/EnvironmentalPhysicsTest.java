package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.world.World;

import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

final class EnvironmentalPhysicsTest {
    @Test
    void emptyAndContractedSurfaceHaveZeroHeight() {
        FakeWorld world = new FakeWorld();
        assertEquals(0, sample(world, 0.5, 0, 0.5).waterHeight());
        world.put(0, 0, 0, state("water", "level", "0"));
        assertEquals(8 / 9.0F - 0.001, sample(world, 0.5, 0, 0.5).waterHeight(), 1e-12);
        assertEquals(0, sample(world, 0.5, 8 / 9.0F, 0.5).waterHeight());
        assertEquals(0, sample(world, -0.3, 0, 0.5).waterHeight());
    }

    @Test
    void stackedWaterUsesFullHeightAndWaterloggedPlantsAreSources() {
        for (String name : new String[]{"kelp", "kelp_plant", "seagrass", "tall_seagrass", "bubble_column"}) {
            FakeWorld world = new FakeWorld();
            world.put(0, 0, 0, state(name));
            assertEquals(8 / 9.0F - 0.001, sample(world, 0.5, 0, 0.5).waterHeight(), 1e-12, name);
        }
        FakeWorld world = new FakeWorld();
        world.put(0, 0, 0, state("oak_slab", "waterlogged", "true"));
        world.put(0, 1, 0, state("water", "level", "0"));
        assertEquals((double) (1 + 8 / 9.0F) - 0.001, sample(world, 0.5, 0, 0.5).waterHeight(), 1e-12);
        assertEquals(0.999, EnvironmentalPhysics.sample(world, new Vector3d(0.5, 0, 0.5), 0.3, 0.5).waterHeight(), 1e-12);
    }

    @Test
    void waterAndLavaUseDifferentPushMultipliers() {
        for (String name : new String[]{"water", "lava"}) {
            FakeWorld world = new FakeWorld();
            world.put(0, 0, 0, state(name, "level", "0"));
            world.put(1, 0, 0, state(name, "level", "7"));
            EnvironmentalPhysics.Environment result = sample(world, 0.5, 0, 0.5);
            Vector3d push = name.equals("water") ? result.waterPush() : result.lavaPush();
            assertEquals(name.equals("water") ? 0.014 : 0.0023333333333333335, push.x, 1e-15);
            assertEquals(0, push.y); assertEquals(0, push.z);
        }
    }

    @Test
    void shallowPushIsScaledWithoutMinimumVelocityCorrection() {
        FakeWorld world = new FakeWorld();
        world.put(0, 0, 0, state("water", "level", "0"));
        world.put(1, 0, 0, state("water", "level", "7"));
        EnvironmentalPhysics.Environment result = sample(world, 0.5, 0.8, 0.5);
        assertEquals((8 / 9.0F - 0.801) * 0.014, result.waterPush().x, 1e-15);
        assertTrue(result.waterPush().length() < 0.0045);
    }

    @Test
    void flowOverDropAndSolidMovementOverrides() {
        FakeWorld world = new FakeWorld();
        world.put(0, 0, 0, state("water", "level", "0"));
        world.put(1, -1, 0, state("water", "level", "0"));
        assertEquals(0.014, sample(world, 0.5, 0, 0.5).waterPush().x, 1e-15);
        world.put(1, 0, 0, state("oak_sign", "waterlogged", "false"));
        assertEquals(0, sample(world, 0.5, 0, 0.5).waterPush().length());
        world.put(1, 0, 0, state("cobweb"));
        assertEquals(0.014, sample(world, 0.5, 0, 0.5).waterPush().x, 1e-15);
    }

    @Test
    void fallingFluidChecksFullSideIncludingBlockAboveButExcludesIceAndLeaves() {
        FakeWorld world = new FakeWorld();
        world.put(0, 0, 0, state("water", "level", "15"));
        assertEquals(0, sample(world, 0.5, 0, 0.5).waterPush().length());
        for (String name : new String[]{"stone", "mud", "soul_sand", "packed_ice", "snow"}) {
            world.put(1, 1, 0, name.equals("snow") ? state(name, "layers", "8") : state(name));
            assertEquals(-0.014, sample(world, 0.5, 0, 0.5).waterPush().y, 1e-15, name);
        }
        for (String name : new String[]{"ice", "frosted_ice", "oak_leaves", "oak_slab"}) {
            world.put(1, 1, 0, state(name));
            assertEquals(0, sample(world, 0.5, 0, 0.5).waterPush().length(), name);
        }
    }

    @Test
    void stationarySamplesRemainInAverageAndOtherFluidDoesNotCreateGradient() {
        FakeWorld world = new FakeWorld();
        world.put(0, 0, 0, state("water", "level", "0"));
        world.put(1, 0, 0, state("water", "level", "0"));
        world.put(2, 0, 0, state("water", "level", "7"));
        assertEquals(0.007, sample(world, 1, 0, 0.5).waterPush().x, 1e-15);
        world.put(2, 0, 0, state("lava", "level", "7"));
        assertEquals(0, sample(world, 1, 0, 0.5).waterPush().length());
    }

    @Test
    void expandedLoadedRegionIncludesCeilingChunkAtBoundary() {
        FakeWorld world = new FakeWorld();
        world.put(14, 0, 8, state("water", "level", "0"));
        world.unloadedX = 1;
        assertEquals(0, sample(world, 14.5, 0, 8.5).waterHeight());
        world.unloadedX = 99;
        assertTrue(sample(world, 14.5, 0, 8.5).waterHeight() > 0);
    }

    @Test
    void allVanillaClimbablesAndTrapdoorAlignmentUseFeetCell() {
        FakeWorld world = new FakeWorld();
        for (String name : new String[]{"ladder", "vine", "scaffolding", "weeping_vines", "weeping_vines_plant",
                "twisting_vines", "twisting_vines_plant", "cave_vines", "cave_vines_plant"}) {
            world.put(-1, 0, -1, state(name));
            EnvironmentalPhysics.Environment result = sample(world, -0.5, 0, -0.5);
            assertTrue(result.climbing(), name);
            assertEquals(name.equals("scaffolding"), result.scaffolding());
        }
        world.put(-1, 0, -1, state("oak_trapdoor", "open", "true", "facing", "north"));
        world.put(-1, -1, -1, state("ladder", "facing", "north"));
        assertTrue(sample(world, -0.5, 0, -0.5).climbing());
        world.put(-1, -1, -1, state("ladder", "facing", "south"));
        assertFalse(sample(world, -0.5, 0, -0.5).climbing());
    }

    private static EnvironmentalPhysics.Environment sample(World world, double x, double y, double z) {
        return EnvironmentalPhysics.sample(world, new Vector3d(x, y, z), 0.3, 1.8);
    }

    private static BlockState state(String name, String... properties) {
        for (int id = 0; id < BlockStateParser.getBlockStateRegistrySize(); id++) {
            BlockState state = BlockStateParser.Instance.parseStateId(id);
            if (!state.blockName().equals(name)) continue;
            boolean matches = true;
            for (int index = 0; index < properties.length; index += 2) {
                matches &= properties[index + 1].equals(state.getProperty(properties[index]));
            }
            if (matches) return state;
        }
        throw new AssertionError("Unknown state " + name);
    }

    private static final class FakeWorld extends World {
        private final Map<String, BlockState> blocks = new HashMap<>();
        private final BlockState air = state("air");
        private int unloadedX = 99;
        void put(int x, int y, int z, BlockState state) { blocks.put(x + ":" + y + ":" + z, state); }
        @Override public boolean chunkLoaded(int x, int z) { return x != unloadedX; }
        @Override public BlockState getBlockStateAt(Vector3d position) {
            return blocks.getOrDefault((int) Math.floor(position.x) + ":" + (int) Math.floor(position.y)
                    + ":" + (int) Math.floor(position.z), air);
        }
    }
}
