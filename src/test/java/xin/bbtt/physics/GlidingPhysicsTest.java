package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3i;

import static org.junit.jupiter.api.Assertions.*;

final class GlidingPhysicsTest {
    private static final double HALF_WIDTH = 0.299;
    private static final double HEIGHT = 0.6;

    private final World air = new World() {
        @Override
        public boolean isBoxColliding(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) {
            return false;
        }
    };

    private PhysicsResult step(PhysicsState state, float pitch, boolean firework) {
        return new GlidingPhysics(0, pitch, firework).step(air, state, HALF_WIDTH, HEIGHT);
    }

    @Test
    void stationaryHandCalculatedTickMovesWithUpdatedVelocity() {
        PhysicsResult result = step(new PhysicsState(new Vector3d(0, 10, 0), new Vector3d(), false), 0, false);
        assertEquals(0, result.velocity().x, 1.0e-9);
        assertEquals(-0.018 * (double) 0.98f, result.velocity().y, 1.0e-12);
        assertEquals(0.0018 * (double) 0.99f, result.velocity().z, 1.0e-12);
        assertEquals(10 + result.velocity().y, result.position().y, 1.0e-12);
        assertEquals(result.velocity().z, result.position().z, 1.0e-12);
        assertFalse(result.onGround());
    }

    @Test
    void rocketHandCalculationChangesNextMotionNotCurrentDisplacement() {
        PhysicsState state = new PhysicsState(new Vector3d(0, 10, 0), new Vector3d(), false);
        PhysicsResult unboosted = step(state, 0, false);
        PhysicsResult boosted = step(state, 0, true);
        assertEquals(unboosted.position(), boosted.position());
        assertEquals(0, boosted.velocity().x, 1.0e-9);
        assertEquals(-0.009 * (double) 0.98f, boosted.velocity().y, 1.0e-12);
        assertEquals(0.85 + 0.0009 * (double) 0.99f, boosted.velocity().z, 1.0e-12);
    }

    @Test
    void diveBuildsHorizontalSpeedAcrossTicks() {
        PhysicsState state = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(), false);
        double firstSpeed = step(state, 35, false).velocity().z;
        for (int tick = 0; tick < 30; tick++) {
            PhysicsResult result = step(state, 35, false);
            state = new PhysicsState(result.position(), result.velocity(), result.onGround());
        }
        assertTrue(state.velocity().z > firstSpeed);
        assertTrue(state.position().y < 100);
    }

    @Test
    void pullingUpTradesHorizontalMotionForAltitude() {
        PhysicsState state = new PhysicsState(new Vector3d(0, 10, 0), new Vector3d(0, 0, 1), false);
        PhysicsResult level = step(state, 0, false);
        PhysicsResult climbing = step(state, -30, false);
        assertTrue(climbing.velocity().y > 0);
        assertTrue(climbing.velocity().y > level.velocity().y);
        assertTrue(climbing.velocity().z < level.velocity().z);
    }

    @Test
    void wallStopsOnlyBlockedHorizontalAxisWithoutStepping() {
        World wall = new World() {
            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return maxX > 1;
            }
        };
        PhysicsResult result = new GlidingPhysics(-90, 0, false).step(wall,
                new PhysicsState(new Vector3d(0, 10, 0), new Vector3d(1, 0, 0), false), HALF_WIDTH, HEIGHT);
        assertTrue(result.collidedX());
        assertFalse(result.collidedZ());
        assertEquals(0, result.position().x);
        assertEquals(0, result.velocity().x);
        assertTrue(result.position().y < 10);
    }

    @Test
    void downwardCollisionLandsAndRetainsAttachedRocketBoost() {
        World floor = new World() {
            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return minY < 0;
            }

            @Override
            public OptionalDouble findHighestCollisionTopExcluding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ, Set<Vector3i> excluded) {
                return minY <= 0 && maxY >= 0 ? OptionalDouble.of(0) : OptionalDouble.empty();
            }
        };
        PhysicsResult result = new GlidingPhysics(0, 0, true).step(floor,
                new PhysicsState(new Vector3d(0, 0.01, 0), new Vector3d(), false), HALF_WIDTH, HEIGHT);
        assertTrue(result.onGround());
        assertEquals(0, result.position().y);
        assertEquals(0, result.velocity().y);
        assertEquals(0.85 + 0.0018 * (double) 0.99f / 2, result.velocity().z, 1.0e-12);
    }

    @Test
    void tickStartSnapsSmallPlayerVelocityBeforeGliding() {
        PhysicsState tiny = new PhysicsState(new Vector3d(0, 20, 0), new Vector3d(0.001, -0.0029, 0.002), false);
        PhysicsState stationary = new PhysicsState(new Vector3d(0, 20, 0), new Vector3d(), false);
        assertEquals(step(stationary, 10, false), step(tiny, 10, false));
        PhysicsState diagonal = new PhysicsState(new Vector3d(0, 20, 0), new Vector3d(0.0025, 0, 0.0025), false);
        assertNotEquals(step(stationary, 10, false), step(diagonal, 10, false));
    }

    @Test
    void inputsAreNotMutatedAndRepeatedStepsAreIdentical() {
        Vector3d position = new Vector3d(0, 20, 0);
        Vector3d velocity = new Vector3d(0.2, -0.1, 0.5);
        PhysicsState state = new PhysicsState(position, velocity, false);
        PhysicsResult first = step(state, -15, true);
        assertEquals(first, step(state, -15, true));
        assertEquals(position, state.position());
        assertEquals(velocity, state.velocity());
        assertEquals(new Vector3d(0, 20, 0), position);
        assertEquals(new Vector3d(0.2, -0.1, 0.5), velocity);
    }
}
