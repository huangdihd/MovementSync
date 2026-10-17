package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

final class GlideRangeTest {
    private double range(double height, double speed, boolean slow) {
        var parameters = new MovementParameters(.1, .13, .42, 0, .08, .6, .3, slow, -1, true);
        return new GlideRange(new PhysicsState(new Vector3d(0, height, 0),
                new Vector3d(0, -.1, speed), false), parameters,
                new float[]{-55, -30, -10, 0, 10}).atHeight(0, 0, false);
    }

    @Test void cachedTrajectoriesMatchFreshPredictionsAcrossUnorderedHeights() {
        var state = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(.2, -.1, .8), false);
        var pitches = new float[]{-55, -30, -10, 0, 10};
        var cached = new GlideRange(state, MovementParameters.DEFAULT, pitches);
        for (double height : new double[]{90, 0, 50, -40, 110, 95, 0}) {
            var fresh = new GlideRange(state, MovementParameters.DEFAULT, pitches);
            assertEquals(fresh.atHeight(height, new int[]{1, 3}, true),
                    cached.atHeight(height, new int[]{1, 3}, true), 1e-9);
            assertEquals(fresh.projectedRange(height, 30, -10), cached.projectedRange(height, 30, -10), 1e-9);
        }
    }

    @Test void rangeDependsOnHeightSpeedAndSlowFalling() {
        assertTrue(range(100, .8, false) > range(10, .8, false));
        assertTrue(range(3, .8, false) > range(3, 0, false));
        assertTrue(range(10, .8, true) > range(10, .8, false));
        assertTrue(range(100, .8, false) > 100 * 5);
    }
}
