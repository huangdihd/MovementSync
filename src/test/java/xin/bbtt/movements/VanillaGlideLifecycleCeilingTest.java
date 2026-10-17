package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import org.cloudburstmc.nbt.NbtMap;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.listeners.RegistryDataListener;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class VanillaGlideLifecycleCeilingTest extends ElytraCommandTestSupport {
    private List<NbtMap> dimensions;
    private List<NbtMap> previous;
    private Field index;
    private int previousIndex;

    @BeforeEach @SuppressWarnings("unchecked")
    void ceilingDimension() throws Exception {
        Field field = RegistryDataListener.class.getDeclaredField("dimensionTypes");
        field.setAccessible(true);
        dimensions = (List<NbtMap>) field.get(null);
        previous = new ArrayList<>(dimensions);
        index = RegistryDataListener.class.getDeclaredField("currentDimensionIndex");
        index.setAccessible(true);
        previousIndex = index.getInt(null);
        dimensions.clear();
        dimensions.add(NbtMap.builder().putBoolean("has_ceiling", true)
                .putInt("min_y", 0).putInt("height", 256).build());
        index.setInt(null, 0);
        movementSync.position.set(new Vector3d(0, 80, 0));
        installWorld(true);
    }

    @AfterEach void restoreDimension() throws Exception {
        dimensions.clear(); dimensions.addAll(previous);
        index.setInt(null, previousIndex);
    }

    private void installWorld(boolean known) throws Exception {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return known; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return y < 0 || Y > 120; }
        };
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true); field.set(movementSync, world);
    }

    @Test void roofBlocksGroundJumpAndDeployment() {
        movementSync.onGround.set(true);
        var lifecycle = new VanillaGlideLifecycle();
        var before = new Vector3d(movementSync.velocity.get());
        lifecycle.init(movementSync);
        assertEquals(before, movementSync.velocity.get());
        boolean[] failed = {false};
        lifecycle.tickTakeoff(movementSync, () -> failed[0] = true);
        assertTrue(failed[0]);
        assertFalse(movementSync.isGliding());
    }

    @Test void dangerousDescentMayRescueWithSafeSharedPrediction() {
        movementSync.onGround.set(false);
        movementSync.velocity.set(new Vector3d(0, -.2, .3));
        assertTrue(VanillaGlideLifecycle.eligibleForAirborneTakeoff(movementSync));
        var lifecycle = new VanillaGlideLifecycle();
        lifecycle.init(movementSync);
        lifecycle.tickTakeoff(movementSync, () -> fail("safe rescue declined"));
        assertTrue(movementSync.isGliding());
    }

    @Test void unknownTerrainCannotAuthorizeRescue() throws Exception {
        installWorld(false);
        movementSync.onGround.set(false);
        movementSync.velocity.set(new Vector3d(0, -.2, .3));
        assertFalse(VanillaGlideLifecycle.eligibleForAirborneTakeoff(movementSync));
    }

    @Test void lavaCannotBeCertifiedAsSafeRescue() throws Exception {
        var lava = java.util.stream.IntStream.range(0, xin.bbtt.Block.BlockStateParser.getBlockStateRegistrySize())
                .mapToObj(xin.bbtt.Block.BlockStateParser.Instance::parseStateId)
                .filter(block -> block.blockName().equals("lava") && "0".equals(block.getProperty("level")))
                .findFirst().orElseThrow();
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d p) { return lava; }
            @Override public boolean isBoxColliding(double x, double y, double z,
                    double X, double Y, double Z) { return Y > 120; }
        };
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true); field.set(movementSync, world);
        movementSync.onGround.set(false);
        movementSync.velocity.set(new Vector3d(0, -.2, .3));
        assertFalse(VanillaGlideLifecycle.eligibleForAirborneTakeoff(movementSync));
        var lifecycle = new VanillaGlideLifecycle();
        lifecycle.init(movementSync);
        boolean[] failed = {false};
        lifecycle.tickTakeoff(movementSync, () -> failed[0] = true);
        assertTrue(failed[0]);
        assertFalse(movementSync.isGliding());
    }

    @Test void ascendingPlayerCannotClaimFallingRescue() {
        movementSync.onGround.set(false);
        movementSync.velocity.set(new Vector3d(0, .2, .3));
        assertFalse(VanillaGlideLifecycle.eligibleForAirborneTakeoff(movementSync));
    }
}
