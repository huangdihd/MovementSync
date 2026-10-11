package xin.bbtt.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

final class WalkingPhysicsTest {
    @Test
    void stateAndResultDoNotExposeMutableVectors() {
        Vector3d position = new Vector3d(0.5, 4, 0.5);
        Vector3d velocity = new Vector3d(0.2, 0.42, 0.1);
        PhysicsState state = new PhysicsState(position, velocity, false);
        PhysicsResult result = new PhysicsResult(position, velocity, true, false);
        position.zero();
        velocity.zero();
        state.position().zero();
        state.velocity().zero();
        result.position().zero();
        result.velocity().zero();

        assertEquals(new Vector3d(0.5, 4, 0.5), state.position());
        assertEquals(new Vector3d(0.2, 0.42, 0.1), state.velocity());
        assertEquals(state.position(), result.position());
        assertEquals(state.velocity(), result.velocity());
        assertFalse(state.onGround());
        assertTrue(result.collidedX());
        assertFalse(result.collidedZ());
    }

    @Test
    void repeatedStepsUseOnlySuppliedStateAndLeaveItUnchanged() {
        World world = new World() {
            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return false;
            }
        };
        PhysicsState state = new PhysicsState(
            new Vector3d(0.5, 4, 0.5), new Vector3d(0.2, 0.42, 0.1), false);
        PhysicsStepper physics = new WalkingPhysics();
        PhysicsResult first = physics.step(world, state, 0.299, 1.8);
        PhysicsResult second = physics.step(world, state, 0.299, 1.8);

        assertEquals(first, second);
        assertEquals(new Vector3d(0.5, 4, 0.5), state.position());
        assertEquals(new Vector3d(0.2, 0.42, 0.1), state.velocity());
        assertEquals(new Vector3d(0.7, 4.42, 0.6), first.position());
        assertEquals((0.42 - 0.08) * 0.9800000190734863D, first.velocity().y);
        assertFalse(first.collidedX());
        assertFalse(first.collidedZ());
    }

    @Test
    void suppliedBodyHeightControlsHeadCollision() {
        World world = new World() {
            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return maxY > 3;
            }
        };
        PhysicsState state = new PhysicsState(
            new Vector3d(0.5, 1, 0.5), new Vector3d(0, 0.42, 0), true);
        PhysicsStepper physics = new WalkingPhysics();
        PhysicsResult standing = physics.step(world, state, 0.299, 1.8);
        PhysicsResult sneaking = physics.step(world, state, 0.299, 1.5);
        PhysicsResult swimming = physics.step(world, state, 0.299, 0.6);

        assertEquals(1, standing.position().y);
        assertEquals(0, standing.velocity().y);
        assertEquals(1.42, sneaking.position().y);
        assertEquals(1.42, swimming.position().y);
    }
}
