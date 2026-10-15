package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;
import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3i;
import static org.junit.jupiter.api.Assertions.*;

class FallPredictionTest {
    private final World floor = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public OptionalDouble findHighestCollisionTopExcluding(double x, double y, double z,
                double xx, double yy, double zz, Set<Vector3i> excluded) {
            return y <= 0 && yy >= 0 ? OptionalDouble.of(0) : OptionalDouble.empty();
        }
    };

    private FallPrediction.Result predict(double height, MovementParameters parameters) {
        return FallPrediction.predict(floor, new PhysicsState(new Vector3d(.5, height, .5),
                new Vector3d(0, -.1, 0), false), parameters);
    }

    @Test void flatGroundHeights() {
        assertFalse(predict(2, MovementParameters.DEFAULT).dangerous());
        assertFalse(predict(3, MovementParameters.DEFAULT).dangerous());
        assertTrue(predict(20, MovementParameters.DEFAULT).dangerous());
    }

    @Test void slowFallingResetsFallDistance() {
        assertFalse(predict(20, new MovementParameters(.1, .13, .42, 0, .08, .6, .3,
                true, -1, true)).dangerous());
    }

    @Test void safeFallAttributeIncludesServerJumpBoostModifier() {
        assertFalse(predict(20, new MovementParameters(.1, .13, .42, .2, .08, .6, .3,
                false, -1, true, 30, 1)).dangerous());
        assertEquals(0, FallPrediction.damage(4, 1, new MovementParameters(.1, .13, .42, .1,
                .08, .6, .3, false, -1, true, 4, 1)));
    }

    @Test void landingInWaterClearsAccumulatedFall() {
        World water = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d p) {
                if (p.y >= 0 && p.y < 5) {
                    for (int id = 0; id < xin.bbtt.Block.BlockStateParser.getBlockStateRegistrySize(); id++) {
                        var block = xin.bbtt.Block.BlockStateParser.Instance.parseStateId(id);
                        if (block.blockName().equals("water") && "0".equals(block.getProperty("level"))) return block;
                    }
                }
                return super.getBlockStateAt(p);
            }
        };
        assertFalse(FallPrediction.predict(water, new PhysicsState(new Vector3d(.5, 20, .5),
                new Vector3d(0, -.1, 0), false, 10), MovementParameters.DEFAULT).dangerous());
    }

    @Test void voidAndUnknownChunksAreDangerous() {
        PhysicsState state = new PhysicsState(new Vector3d(.5, 10, .5), new Vector3d(0, -.1, 0), false);
        assertTrue(FallPrediction.predict(new World(), state, MovementParameters.DEFAULT).dangerous());
        World empty = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
        };
        assertTrue(FallPrediction.predict(empty, state, MovementParameters.DEFAULT).dangerous());
    }
}
