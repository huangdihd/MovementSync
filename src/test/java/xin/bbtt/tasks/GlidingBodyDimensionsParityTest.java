package xin.bbtt.tasks;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.GlidingPhysics;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Player FALL_FLYING uses EntityDimensions(0.6F, 0.6F), without a collision inset. */
final class GlidingBodyDimensionsParityTest {
    @Test
    void liveGlideBoxMatchesVanillaFloatDimensionsAndWallContact() throws Exception {
        MovementSync previous = MovementSync.INSTANCE;
        try {
            MovementSync sync = new MovementSync();
            sync.setGliding(true);
            var dimensionsMethod = updateMotionTask.class.getDeclaredMethod("getBodyDimensions");
            dimensionsMethod.setAccessible(true);
            Object dimensions = dimensionsMethod.invoke(new updateMotionTask());
            var widthMethod = dimensions.getClass().getDeclaredMethod("halfWidth");
            var heightMethod = dimensions.getClass().getDeclaredMethod("height");
            widthMethod.setAccessible(true);
            heightMethod.setAccessible(true);
            double halfWidth = (double) widthMethod.invoke(dimensions);
            double height = (double) heightMethod.invoke(dimensions);
            assertEquals((double) (0.6f / 2), halfWidth);
            assertEquals((double) 0.6f, height);

            World wall = new World() {
                @Override public boolean chunkLoaded(int x, int z) { return true; }
                @Override public int getBlockAt(Vector3d p) { return p.x >= 1 ? 1 : 0; }
            };
            var state = new PhysicsState(new Vector3d(0.5, 10, 0.5), new Vector3d(0.5, 0, 0), false);
            var physics = new GlidingPhysics(-90, 0, false);
            var actual = physics.step(wall, state, halfWidth, height);
            assertTrue(actual.collidedX());
            assertEquals(1 - (double) (0.6f / 2), actual.position().x, 1.0e-12);
            // The previous inset advances one extra milliblock into a vanilla wall.
            var inset = physics.step(wall, state, 0.299, 0.6);
            assertEquals((double) (0.6f / 2) - 0.299,
                    inset.position().x - actual.position().x, 1.0e-12);
        } finally {
            MovementSync.INSTANCE = previous;
        }
    }
}
