package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.assertEquals;

final class GlidingGravityTest {
    private final World air = new World() {
        @Override
        public boolean isBoxColliding(double minimumX, double minimumY, double minimumZ,
                double maximumX, double maximumY, double maximumZ) {
            return false;
        }
    };

    @Test
    void gravityAttributeScalesLevelFlightLift() {
        // LivingEntity.calcGlidingVelocity: gravity * (-1 + .75), then .1 lift and .98F drag.
        PhysicsResult result = step(0.16, false, 0);
        assertEquals(-0.16 * 0.25 * 0.9 * (double) 0.98f, result.velocity().y, 1.0e-12);
    }

    @Test
    void slowFallingCapsGravityOnlyWhileDescending() {
        // LivingEntity.getEffectiveGravity: vy <= 0 ? min(gravity, .01) : gravity.
        assertEquals(-0.01 * 0.25 * 0.9 * (double) 0.98f,
                step(0.08, true, 0).velocity().y, 1.0e-12);
        assertEquals((0.2 - 0.08 * 0.25) * (double) 0.98f,
                step(0.08, true, 0.2).velocity().y, 1.0e-12);
    }

    @Test
    void slowFallingDoesNotIncreaseLowGravity() {
        assertEquals(step(0.005, false, 0).velocity(), step(0.005, true, 0).velocity());
    }

    private PhysicsResult step(double gravity, boolean slowFalling, double verticalVelocity) {
        return new GlidingPhysics(0, 0, false, gravity, slowFalling).step(air,
                new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, verticalVelocity, 0), false),
                0.299, 0.6);
    }
}
