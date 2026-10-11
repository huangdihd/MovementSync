package xin.bbtt.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

final class WalkingPhysicsTest {
    private static final double GROUND_ACCELERATION = 0.098 * 0.21600002 / (0.6 * 0.6 * 0.6);
    private static final double GROUND_DRAG = 0.6 * 0.91;
    private static final World EMPTY_WORLD = new World() {
        @Override
        public boolean isBoxColliding(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) {
            return false;
        }
    };

    @Test
    void forwardStartsAtVanillaAccelerationAndConvergesToWalkingSpeed() {
        WalkingPhysics physics = new WalkingPhysics(input(true, false, false, false, false), 0);
        PhysicsState state = state(new Vector3d(), true);
        PhysicsResult first = physics.step(EMPTY_WORLD, state, 0.299, 1.8);
        assertEquals(0.098, first.position().z, 1.0e-7);
        assertEquals(GROUND_ACCELERATION * GROUND_DRAG, first.velocity().z, 1.0e-12);
        for (int tick = 0; tick < 100; tick++) {
            PhysicsResult result = physics.step(EMPTY_WORLD, state, 0.299, 1.8);
            double distance = result.position().z - state.position().z;
            if (tick == 99) assertEquals(0.2159, distance, 0.0001);
            state = new PhysicsState(result.position(), result.velocity(), true);
        }
    }

    @Test
    void walkingOffSingleBlockStaysGroundedForSupportedTickThenFalls() {
        double gravityVelocity = -0.08 * 0.9800000190734863D;
        World world = new World() {
            @Override
            public java.util.OptionalDouble findHighestCollisionTopExcluding(
                    double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ,
                    java.util.Set<org.joml.Vector3i> excludedBlocks) {
                boolean overlapsBlock = minX < 1 && maxX > 0 && minZ < 1 && maxZ > 0;
                return overlapsBlock && minY <= -56 && maxY >= -56
                    ? java.util.OptionalDouble.of(-56) : java.util.OptionalDouble.empty();
            }

            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return minX < 1 && maxX > 0 && minZ < 1 && maxZ > 0
                    && minY < -56 && maxY > -57;
            }
        };
        WalkingPhysics physics = new WalkingPhysics(input(true, false, false, false, false), 0);
        PhysicsResult landing = new WalkingPhysics().step(world, new PhysicsState(
            new Vector3d(0.5, -55.9, 0.5), new Vector3d(0, -0.2, 0), false), 0.299, 1.8);
        assertEquals(-56, landing.position().y);
        assertEquals(gravityVelocity, landing.velocity().y);
        assertTrue(landing.onGround());

        PhysicsState state = new PhysicsState(landing.position(), landing.velocity(), true);
        boolean leftEdge = false;
        for (int tick = 0; tick < 20; tick++) {
            PhysicsResult result = physics.step(world, state, 0.299, 1.8);
            if (result.position().z - 0.299 >= 1) {
                // Downward travel is clipped while the tick still starts over support.
                assertEquals(-56, result.position().y);
                assertEquals(gravityVelocity, result.velocity().y);
                assertTrue(result.onGround());
                PhysicsResult falling = physics.step(world, new PhysicsState(
                    result.position(), result.velocity(), result.onGround()), 0.299, 1.8);
                assertEquals(-56 + gravityVelocity, falling.position().y, 1.0e-12);
                assertEquals((gravityVelocity - 0.08) * 0.9800000190734863D,
                    falling.velocity().y, 1.0e-12);
                assertFalse(falling.onGround());
                assertTrue(falling.position().z > result.position().z);
                leftEdge = true;
                break;
            }
            assertEquals(-56, result.position().y);
            assertEquals(gravityVelocity, result.velocity().y);
            assertTrue(result.onGround());
            state = new PhysicsState(result.position(), result.velocity(), result.onGround());
        }
        assertTrue(leftEdge, "Forward movement must leave the single block");
    }

    @Test
    void releasingKeysMovesBeforeDragThenEventuallyStops() {
        PhysicsState state = state(new Vector3d(0.1, 0, -0.1), true);
        WalkingPhysics physics = new WalkingPhysics();
        PhysicsResult first = physics.step(EMPTY_WORLD, state, 0.299, 1.8);
        assertEquals(0.1, first.position().x, 1.0e-12);
        assertEquals(-0.1, first.position().z, 1.0e-12);
        assertEquals(0.1 * GROUND_DRAG, first.velocity().x, 1.0e-12);
        for (int tick = 0; tick < 20; tick++) {
            PhysicsResult result = physics.step(EMPTY_WORLD, state, 0.299, 1.8);
            state = new PhysicsState(result.position(), result.velocity(), true);
        }
        assertEquals(0, state.velocity().x);
        assertEquals(0, state.velocity().z);
    }

    @Test
    void airAccelerationAndDragUseTickStartGroundState() {
        for (boolean sprint : new boolean[]{false, true}) {
            WalkingPhysics physics = new WalkingPhysics(input(true, false, false, false, sprint), 0);
            PhysicsResult result = physics.step(EMPTY_WORLD, state(new Vector3d(), false), 0.299, 1.8);
            double acceleration = 0.98 * (sprint ? 0.026 : 0.02);
            assertEquals(acceleration, result.position().z, 1.0e-12);
            assertEquals(acceleration * 0.91, result.velocity().z, 1.0e-12);
            assertEquals(0, result.position().y);
            assertEquals(-0.08 * 0.9800000190734863D, result.velocity().y);
        }
    }

    @Test
    void yawAndDiagonalNormalizationKeepInputMagnitude() {
        WalkingPhysics forward = new WalkingPhysics(input(true, false, false, false, false), 90);
        PhysicsResult rotated = forward.step(EMPTY_WORLD, state(new Vector3d(), true), 0.299, 1.8);
        assertEquals(-GROUND_ACCELERATION, rotated.position().x, 1.0e-12);
        assertEquals(0, rotated.position().z, 1.0e-12);
        WalkingPhysics diagonal = new WalkingPhysics(input(true, true, false, false, false), 0);
        PhysicsResult result = diagonal.step(EMPTY_WORLD, state(new Vector3d(), true), 0.299, 1.8);
        assertEquals(GROUND_ACCELERATION / Math.sqrt(2), result.position().x, 1.0e-12);
        assertEquals(result.position().x, result.position().z);
        WalkingPhysics opposed = new WalkingPhysics(new PlayerInput(true, true, true, true, false, false, false), 0);
        assertEquals(new Vector3d(), opposed.step(EMPTY_WORLD, state(new Vector3d(), true), 0.299, 1.8).position());
        WalkingPhysics backwardRight = new WalkingPhysics(new PlayerInput(false, true, false, true, false, false, false), 0);
        Vector3d opposite = backwardRight.step(EMPTY_WORLD, state(new Vector3d(), true), 0.299, 1.8).position();
        assertEquals(-result.position().x, opposite.x, 1.0e-12);
        assertEquals(-result.position().z, opposite.z, 1.0e-12);
    }

    @Test
    void sneakAndSprintScaleGroundAcceleration() {
        PhysicsState state = state(new Vector3d(), true);
        PhysicsResult sneak = new WalkingPhysics(input(true, false, false, true, false), 0)
            .step(EMPTY_WORLD, state, 0.299, 1.8);
        PhysicsResult sprint = new WalkingPhysics(input(true, false, false, false, true), 0)
            .step(EMPTY_WORLD, state, 0.299, 1.8);
        assertEquals(GROUND_ACCELERATION * 0.3, sneak.position().z, 1.0e-12);
        assertEquals(GROUND_ACCELERATION * 1.3, sprint.position().z, 1.0e-12);
    }

    @Test
    void sprintJumpAddsYawImpulseAndUsesGroundDrag() {
        PlayerInput input = input(true, false, true, false, true);
        PhysicsState state = state(new Vector3d(), true);
        PhysicsResult jump = new WalkingPhysics(input, 90).step(EMPTY_WORLD, state, 0.299, 1.8);
        assertEquals(0.42, jump.position().y);
        assertEquals(-(0.2 + GROUND_ACCELERATION * 1.3), jump.position().x, 1.0e-12);
        assertEquals(jump.position().x * GROUND_DRAG, jump.velocity().x, 1.0e-12);
        assertEquals((0.42 - 0.08) * 0.9800000190734863D, jump.velocity().y);
        PhysicsResult blocked = new WalkingPhysics(input, 90, false).step(EMPTY_WORLD, state, 0.299, 1.8);
        assertEquals(0, blocked.position().y);
        assertEquals(-GROUND_ACCELERATION * 1.3, blocked.position().x, 1.0e-12);
        PhysicsResult airborne = new WalkingPhysics(input, 90).step(EMPTY_WORLD, state(new Vector3d(), false), 0.299, 1.8);
        assertEquals(0, airborne.position().y);
        assertEquals(-0.026 * 0.98, airborne.position().x, 1.0e-12);
        PhysicsResult standing = new WalkingPhysics(input(false, false, true, false, false), 0)
            .step(EMPTY_WORLD, state, 0.299, 1.8);
        assertEquals(new Vector3d(0, 0.42, 0), standing.position());
    }

    @Test
    void nearZeroThresholdIsStrictAndAppliesBeforeInputOnEveryAxis() {
        for (double sign : new double[]{-1, 1}) {
            PhysicsResult snapped = new WalkingPhysics().step(EMPTY_WORLD,
                state(new Vector3d(sign * 0.002999), false), 0.299, 1.8);
            assertEquals(new Vector3d(), snapped.position());
            PhysicsResult boundary = new WalkingPhysics().step(EMPTY_WORLD,
                state(new Vector3d(sign * 0.003), false), 0.299, 1.8);
            assertEquals(new Vector3d(sign * 0.003), boundary.position());
        }
        PhysicsResult accelerated = new WalkingPhysics(input(true, false, false, false, false), 0)
            .step(EMPTY_WORLD, state(new Vector3d(0, 0, 0.002), true), 0.299, 1.8);
        assertEquals(GROUND_ACCELERATION, accelerated.position().z, 1.0e-12);
    }

    @Test
    void inputDrivenStepIsRepeatableAndDoesNotMutateState() {
        PhysicsState state = state(new Vector3d(0.1, -0.1, 0.2), true);
        WalkingPhysics physics = new WalkingPhysics(input(true, true, true, false, true), 37);
        PhysicsResult first = physics.step(EMPTY_WORLD, state, 0.299, 1.8);
        assertEquals(first, physics.step(EMPTY_WORLD, state, 0.299, 1.8));
        assertEquals(new Vector3d(), state.position());
        assertEquals(new Vector3d(0.1, -0.1, 0.2), state.velocity());
    }

    private static PlayerInput input(boolean forward, boolean left, boolean jump, boolean sneak, boolean sprint) {
        return new PlayerInput(forward, false, left, false, jump, sneak, sprint);
    }

    private static PhysicsState state(Vector3d velocity, boolean onGround) {
        return new PhysicsState(new Vector3d(), velocity, onGround);
    }

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
        assertEquals(-0.08 * 0.9800000190734863D, standing.velocity().y);
        assertEquals(1.42, sneaking.position().y);
        assertEquals(1.42, swimming.position().y);
    }
}
