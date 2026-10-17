package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class GlideRangeFireworkTest {
    private final PhysicsState initial = new PhysicsState(new Vector3d(0, 12, 0),
            new Vector3d(0, -.1, .8), false);
    private final MovementParameters parameters = MovementParameters.DEFAULT;

    private GlideRange range(PhysicsState state) {
        return new GlideRange(state, parameters, new float[]{0});
    }

    private Vector3d crossing(PhysicsState initial, float yaw, int boostTicks) {
        var world = new World();
        PhysicsState state = initial;
        for (int tick = 0; tick < 6000; tick++) {
            var result = new GlidingPhysics(yaw, 0, tick < boostTicks,
                    parameters.gravity(), parameters.slowFalling()).step(world, state, .299, .6);
            if (result.position().y <= 0 && result.velocity().y < 0) {
                double fraction = state.position().y / (state.position().y - result.position().y);
                return state.position().lerp(result.position(), fraction);
            }
            state = new PhysicsState(result.position(), result.velocity(), false);
        }
        throw new AssertionError("trajectory did not cross height zero");
    }

    @Test void countOnlyMeansFlightOneAndIncludesExpiryTick() {
        var range = range(initial);
        assertEquals(crossing(initial, 0, 21).z, range.atHeight(0, 1, false), 1e-9);
        assertEquals(range.atHeight(0, new int[]{1, 1}, false), range.atHeight(0, 2, false), 1e-9);
        assertNotEquals(crossing(initial, 0, 20).z, range.atHeight(0, 1, false), 1e-6);
    }

    @Test void componentDurationsUseRandomMinimaAndAttachedCreditsOnlyOneTick() {
        var range = range(initial);
        assertEquals(crossing(initial, 0, 11).z, range.atHeight(0, new int[]{0}, false), 1e-9);
        assertEquals(crossing(initial, 0, 31).z, range.atHeight(0, new int[]{2}, false), 1e-9);
        assertEquals(crossing(initial, 0, 62).z, range.atHeight(0, new int[]{1, 3}, false), 1e-9);
        assertEquals(crossing(initial, 0, 63).z, range.atHeight(0, new int[]{1, 3}, true), 1e-9);
        assertEquals(crossing(initial, 0, 1).z, range.atHeight(0, new int[0], true), 1e-9);
        assertTrue(range.atHeight(0, new int[]{3}, false) > range.atHeight(0, new int[]{1}, false));
    }

    @Test void projectedRangeUsesRequestedYawAndExcludesLateralTravelAndRockets() {
        var lateral = new PhysicsState(new Vector3d(5, 1, 7), new Vector3d(2, -.2, .3), false);
        Vector3d expected = crossing(lateral, 0, 0).sub(lateral.position());
        assertEquals(expected.z, range(lateral).projectedRange(0, 0, 0), 1e-9);
        assertTrue(Math.hypot(expected.x, expected.z) > expected.z + .1);
        Vector3d turned = crossing(lateral, 90, 0).sub(lateral.position());
        assertEquals(-turned.x, range(lateral).projectedRange(0, 90, 0), 1e-9);
        assertEquals(crossing(initial, 0, 0).z, range(initial).projectedRange(0, 0, 0), 1e-9);
    }

    @Test void repeatedQueriesDoNotMutateInitialStateAndUnreachableHeightReturnsZero() {
        var range = range(initial);
        double first = range.atHeight(0, new int[]{1}, false);
        range.atHeight(5, 0, false);
        assertEquals(first, range.atHeight(0, new int[]{1}, false));
        assertEquals(new Vector3d(0, 12, 0), initial.position());
        assertEquals(0, range.projectedRange(1000, 0, 0));
    }
}
