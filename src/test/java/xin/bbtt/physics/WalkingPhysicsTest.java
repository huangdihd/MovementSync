package xin.bbtt.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

final class WalkingPhysicsTest {
    private static final double PLAYER_HALF_WIDTH = (double) 0.6f / 2;
    private static final double GROUND_ACCELERATION = ((double) 0.1F * 0.98) * 0.21600002 / (0.6 * 0.6 * 0.6);
    private static final double GROUND_DRAG = 0.6 * 0.91;
    private static final World EMPTY_WORLD = new World() {
            @Override public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }
        @Override
        public boolean isBoxColliding(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) {
            return false;
        }
    };

    @Test
    void clearedGlidingFlagRetainsCrawlingInputUntilPoseUpdate() {
        PlayerInput forward = input(true, false, false, false, true);
        WalkingPhysics physics = new WalkingPhysics(forward, 0);
        PhysicsState landing = new PhysicsState(new Vector3d(), new Vector3d(), true, 0, PlayerPose.GLIDING);
        PhysicsResult slow = physics.step(EMPTY_WORLD, landing, PLAYER_HALF_WIDTH, 0.6);
        assertEquals(GROUND_ACCELERATION * MovementParameters.DEFAULT.sneakingSpeed(), slow.position().z, 1.0e-12);
        PlayerPose next = landing.pose().afterTravel(EMPTY_WORLD, slow.position(), false, false, false);
        assertEquals(PlayerPose.STANDING, next);
        PhysicsResult normal = physics.step(EMPTY_WORLD,
                new PhysicsState(new Vector3d(), new Vector3d(), true, 0, next), PLAYER_HALF_WIDTH, 1.8);
        assertEquals(GROUND_ACCELERATION * (1 + (double) 0.3f), normal.position().z, 1.0e-12);
        assertFalse(PlayerPose.GLIDING.shouldSlowDown(true, false, false));
        assertFalse(PlayerPose.GLIDING.shouldSlowDown(false, true, false));
        assertTrue(PlayerPose.CROUCHING.shouldSlowDown(false, false, true));
    }

    @Test
    void sneakInputChangesPoseAfterTravelNotBeforeAcceleration() {
        World floor = supportWorld(-2, 2, -2, 2, 0);
        PlayerInput keys = input(true, false, false, true, false);
        PhysicsResult first = new WalkingPhysics(keys, -90).step(floor,
                state(new Vector3d(), true), PLAYER_HALF_WIDTH, 1.8);
        assertEquals(GROUND_ACCELERATION, first.position().x, 1.0e-12);
        assertEquals(0, first.position().z);
        assertEquals(PlayerPose.CROUCHING, first.pose());
        PhysicsResult second = new WalkingPhysics(keys, -90).step(floor,
                new PhysicsState(first.position(), new Vector3d(first.velocity().x, 0, 0), true, 0, first.pose()),
                PLAYER_HALF_WIDTH, 1.5);
        assertEquals(first.velocity().x + GROUND_ACCELERATION * 0.3,
                second.position().x - first.position().x, 1.0e-12);
    }

    @Test
    void forwardStartsAtVanillaAccelerationAndConvergesToWalkingSpeed() {
        WalkingPhysics physics = new WalkingPhysics(input(true, false, false, false, false), 0);
        PhysicsState state = state(new Vector3d(), true);
        PhysicsResult first = physics.step(EMPTY_WORLD, state, PLAYER_HALF_WIDTH, 1.8);
        assertEquals(0.098, first.position().z, 1.0e-7);
        assertEquals(GROUND_ACCELERATION * GROUND_DRAG, first.velocity().z, 1.0e-12);
        for (int tick = 0; tick < 100; tick++) {
            PhysicsResult result = physics.step(EMPTY_WORLD, state, PLAYER_HALF_WIDTH, 1.8);
            double distance = result.position().z - state.position().z;
            if (tick == 99) assertEquals(0.2159, distance, 0.0001);
            state = new PhysicsState(result.position(), result.velocity(), true);
        }
    }

    @Test
    void walkingOffSingleBlockStaysGroundedForSupportedTickThenFalls() {
        double gravityVelocity = -0.08 * 0.9800000190734863D;
        World world = new World() {
            @Override public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }
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
            new Vector3d(0.5, -55.9, 0.5), new Vector3d(0, -0.2, 0), false), PLAYER_HALF_WIDTH, 1.8);
        assertEquals(-56, landing.position().y);
        assertEquals(gravityVelocity, landing.velocity().y);
        assertTrue(landing.onGround());

        PhysicsState state = new PhysicsState(landing.position(), landing.velocity(), true);
        boolean leftEdge = false;
        for (int tick = 0; tick < 20; tick++) {
            PhysicsResult result = physics.step(world, state, PLAYER_HALF_WIDTH, 1.8);
            if (result.position().z - PLAYER_HALF_WIDTH >= 1) {
                // Downward travel is clipped while the tick still starts over support.
                assertEquals(-56, result.position().y);
                assertEquals(gravityVelocity, result.velocity().y);
                assertTrue(result.onGround());
                PhysicsResult falling = physics.step(world, new PhysicsState(
                    result.position(), result.velocity(), result.onGround()), PLAYER_HALF_WIDTH, 1.8);
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
        PhysicsResult first = physics.step(EMPTY_WORLD, state, PLAYER_HALF_WIDTH, 1.8);
        assertEquals(0.1, first.position().x, 1.0e-12);
        assertEquals(-0.1, first.position().z, 1.0e-12);
        assertEquals(0.1 * GROUND_DRAG, first.velocity().x, 1.0e-12);
        for (int tick = 0; tick < 20; tick++) {
            PhysicsResult result = physics.step(EMPTY_WORLD, state, PLAYER_HALF_WIDTH, 1.8);
            state = new PhysicsState(result.position(), result.velocity(), true);
        }
        assertEquals(0, state.velocity().x);
        assertEquals(0, state.velocity().z);
    }

    @Test
    void airAccelerationAndDragUseTickStartGroundState() {
        for (boolean sprint : new boolean[]{false, true}) {
            WalkingPhysics physics = new WalkingPhysics(input(true, false, false, false, sprint), 0);
            PhysicsResult result = physics.step(EMPTY_WORLD, state(new Vector3d(), false), PLAYER_HALF_WIDTH, 1.8);
            double acceleration = 0.98 * (sprint ? 0.026 : 0.02);
            assertEquals(acceleration, result.position().z, 1.0e-12);
            assertEquals(acceleration * 0.91, result.velocity().z, 1.0e-12);
            assertEquals(0, result.position().y);
            assertEquals(-0.08 * 0.9800000190734863D, result.velocity().y);
        }
    }

    @Test
    void yawAndSquareDiagonalInputPreserveDirection() {
        WalkingPhysics forward = new WalkingPhysics(input(true, false, false, false, false), 90);
        PhysicsResult rotated = forward.step(EMPTY_WORLD, state(new Vector3d(), true), PLAYER_HALF_WIDTH, 1.8);
        assertEquals(-GROUND_ACCELERATION, rotated.position().x, 1.0e-12);
        assertEquals(0, rotated.position().z, 1.0e-12);
        WalkingPhysics diagonal = new WalkingPhysics(input(true, true, false, false, false), 0);
        PhysicsResult result = diagonal.step(EMPTY_WORLD, state(new Vector3d(), true), PLAYER_HALF_WIDTH, 1.8);
        assertEquals(GROUND_ACCELERATION / (0.98 * Math.sqrt(2)), result.position().x, 1.0e-12);
        assertEquals(GROUND_ACCELERATION / 0.98,
            Math.hypot(result.position().x, result.position().z), 1.0e-12);
        assertEquals(result.position().x, result.position().z);
        WalkingPhysics opposed = new WalkingPhysics(new PlayerInput(true, true, true, true, false, false, false), 0);
        assertEquals(new Vector3d(), opposed.step(EMPTY_WORLD, state(new Vector3d(), true), PLAYER_HALF_WIDTH, 1.8).position());
        WalkingPhysics backwardRight = new WalkingPhysics(new PlayerInput(false, true, false, true, false, false, false), 0);
        Vector3d opposite = backwardRight.step(EMPTY_WORLD, state(new Vector3d(), true), PLAYER_HALF_WIDTH, 1.8).position();
        assertEquals(-result.position().x, opposite.x, 1.0e-12);
        assertEquals(-result.position().z, opposite.z, 1.0e-12);
    }

    @Test
    void diagonalSquareInputCapsWalkingButPreservesSneakingAxes() {
        World floor = supportWorld(-2, 2, -2, 2, 0);
        for (boolean onGround : new boolean[]{false, true}) {
            for (boolean sneak : new boolean[]{false, true}) {
                for (boolean sprint : new boolean[]{false, true}) {
                    double acceleration = onGround
                        ? GROUND_ACCELERATION / 0.98 * (sprint && !sneak ? 1 + (double) 0.3F : 1)
                        : (sprint && !sneak ? 0.026 : 0.02);
                    double expectedAxis = acceleration * (sneak ? 0.294 : 1 / Math.sqrt(2));
                    double drag = onGround ? GROUND_DRAG : 0.91;
                    PhysicsResult result = new WalkingPhysics(input(true, true, false, sneak, sprint), 0)
                        .step(floor, new PhysicsState(new Vector3d(), new Vector3d(), onGround, 0,
                                sneak ? PlayerPose.CROUCHING : PlayerPose.STANDING), PLAYER_HALF_WIDTH, 1.8);
                    assertEquals(expectedAxis, result.position().x, 1.0e-12);
                    assertEquals(expectedAxis, result.position().z, 1.0e-12);
                    assertEquals(expectedAxis * drag, result.velocity().x, 1.0e-12);
                    assertEquals(expectedAxis * drag, result.velocity().z, 1.0e-12);

                    PhysicsResult rotated = new WalkingPhysics(input(true, true, false, sneak, sprint), 45)
                        .step(floor, new PhysicsState(new Vector3d(), new Vector3d(), onGround, 0,
                                sneak ? PlayerPose.CROUCHING : PlayerPose.STANDING), PLAYER_HALF_WIDTH, 1.8);
                    assertEquals(0, rotated.position().x, 1.0e-12);
                    assertEquals(expectedAxis * Math.sqrt(2), rotated.position().z, 1.0e-12);
                }
            }
        }
    }

    @Test
    void sneakAndSprintScaleGroundAcceleration() {
        PhysicsState state = state(new Vector3d(), true);
        PhysicsResult sneak = new WalkingPhysics(input(true, false, false, true, false), 0)
            .step(supportWorld(-1, 1, -1, 1, 0),
                    new PhysicsState(state.position(), state.velocity(), true, 0, PlayerPose.CROUCHING),
                    PLAYER_HALF_WIDTH, 1.8);
        PhysicsResult sprint = new WalkingPhysics(input(true, false, false, false, true), 0)
            .step(EMPTY_WORLD, state, PLAYER_HALF_WIDTH, 1.8);
        assertEquals(GROUND_ACCELERATION * 0.3, sneak.position().z, 1.0e-12);
        assertEquals(GROUND_ACCELERATION * (1 + (double) 0.3F), sprint.position().z, 1.0e-12);
    }

    @Test
    void sprintJumpAddsYawImpulseAndUsesGroundDrag() {
        PlayerInput input = input(true, false, true, false, true);
        PhysicsState state = state(new Vector3d(), true);
        PhysicsResult jump = new WalkingPhysics(input, 90).step(EMPTY_WORLD, state, PLAYER_HALF_WIDTH, 1.8);
        assertEquals((double) 0.42F, jump.position().y);
        assertEquals(-(0.2 + GROUND_ACCELERATION * (1 + (double) 0.3F)), jump.position().x, 1.0e-12);
        assertEquals(jump.position().x * GROUND_DRAG, jump.velocity().x, 1.0e-12);
        assertEquals(((double) 0.42F - 0.08) * 0.9800000190734863D, jump.velocity().y);
        PhysicsResult blocked = new WalkingPhysics(input, 90, false).step(EMPTY_WORLD, state, PLAYER_HALF_WIDTH, 1.8);
        assertEquals(0, blocked.position().y);
        assertEquals(-GROUND_ACCELERATION * (1 + (double) 0.3F), blocked.position().x, 1.0e-12);
        PhysicsResult airborne = new WalkingPhysics(input, 90).step(EMPTY_WORLD, state(new Vector3d(), false), PLAYER_HALF_WIDTH, 1.8);
        assertEquals(0, airborne.position().y);
        assertEquals(-0.026 * 0.98, airborne.position().x, 1.0e-12);
        PhysicsResult standing = new WalkingPhysics(input(false, false, true, false, false), 0)
            .step(EMPTY_WORLD, state, PLAYER_HALF_WIDTH, 1.8);
        assertEquals(new Vector3d(0, (double) 0.42F, 0), standing.position());
    }

    @Test
    void nearZeroThresholdIsStrictAndAppliesBeforeInput() {
        for (double sign : new double[]{-1, 1}) {
            PhysicsResult snapped = new WalkingPhysics().step(EMPTY_WORLD,
                state(new Vector3d(sign * 0.002999, sign * 0.002999, 0), false), PLAYER_HALF_WIDTH, 1.8);
            assertEquals(new Vector3d(), snapped.position());
            PhysicsResult boundary = new WalkingPhysics().step(EMPTY_WORLD,
                state(new Vector3d(sign * 0.003, sign * 0.003, 0), false), PLAYER_HALF_WIDTH, 1.8);
            assertEquals(new Vector3d(sign * 0.003, sign * 0.003, 0), boundary.position());
        }
        PhysicsResult accelerated = new WalkingPhysics(input(true, false, false, false, false), 0)
            .step(EMPTY_WORLD, state(new Vector3d(0, 0, 0.002), true), PLAYER_HALF_WIDTH, 1.8);
        assertEquals(GROUND_ACCELERATION, accelerated.position().z, 1.0e-12);
    }

    @Test
    void inputDrivenStepIsRepeatableAndDoesNotMutateState() {
        PhysicsState state = state(new Vector3d(0.1, -0.1, 0.2), true);
        WalkingPhysics physics = new WalkingPhysics(input(true, true, true, false, true), 37);
        PhysicsResult first = physics.step(EMPTY_WORLD, state, PLAYER_HALF_WIDTH, 1.8);
        assertEquals(first, physics.step(EMPTY_WORLD, state, PLAYER_HALF_WIDTH, 1.8));
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
            @Override public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }
            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return false;
            }
        };
        PhysicsState state = new PhysicsState(
            new Vector3d(0.5, 4, 0.5), new Vector3d(0.2, 0.42, 0.1), false);
        PhysicsStepper physics = new WalkingPhysics();
        PhysicsResult first = physics.step(world, state, PLAYER_HALF_WIDTH, 1.8);
        PhysicsResult second = physics.step(world, state, PLAYER_HALF_WIDTH, 1.8);

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
            @Override public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }
            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return maxY > 3;
            }
        };
        PhysicsState state = new PhysicsState(
            new Vector3d(0.5, 1, 0.5), new Vector3d(0, 0.42, 0), true);
        PhysicsStepper physics = new WalkingPhysics();
        PhysicsResult standing = physics.step(world, state, PLAYER_HALF_WIDTH, 1.8);
        PhysicsResult sneaking = physics.step(world, state, PLAYER_HALF_WIDTH, 1.5);
        PhysicsResult swimming = physics.step(world, state, PLAYER_HALF_WIDTH, 0.6);

        assertEquals(1.2, standing.position().y, 1.0e-12);
        assertEquals(-0.08 * 0.9800000190734863D, standing.velocity().y);
        assertEquals(1.42, sneaking.position().y);
        assertEquals(1.42, swimming.position().y);
    }
    @Test
    void sneakingClipsBothSignedAxesWithoutClippingMomentum() {
        World world = supportWorld(0, 1, 0, 1, 0);
        WalkingPhysics physics = new WalkingPhysics(input(false, false, false, true, false), 0);
        for (boolean xAxis : new boolean[]{false, true}) {
            for (double sign : new double[]{-1, 1}) {
                Vector3d position = new Vector3d(0.5, 0, 0.5);
                Vector3d velocity = xAxis ? new Vector3d(sign * 0.2, 0, 0) : new Vector3d(0, 0, sign * 0.2);
                if (xAxis) position.x = sign > 0 ? 1.25 : -0.25;
                else position.z = sign > 0 ? 1.25 : -0.25;
                PhysicsState state = new PhysicsState(position, velocity, true);
                PhysicsResult result = physics.step(world, state, PLAYER_HALF_WIDTH, 1.8);
                assertEquals(position.x + (xAxis ? sign * 0.05 : 0), result.position().x, 1.0e-12);
                assertEquals(position.z + (xAxis ? 0 : sign * 0.05), result.position().z, 1.0e-12);
                assertEquals(velocity.x * GROUND_DRAG, result.velocity().x, 1.0e-12);
                assertEquals(velocity.z * GROUND_DRAG, result.velocity().z, 1.0e-12);
                assertFalse(result.collidedX());
                assertFalse(result.collidedZ());
                assertEquals(result, physics.step(world, state, PLAYER_HALF_WIDTH, 1.8));
                assertEquals(position, state.position());
                assertEquals(velocity, state.velocity());
            }
        }
    }

    @Test
    void sneakingClipsDiagonalWhenEachAxisAloneHasSupport() {
        World world = new World() {
            @Override public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }
            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return minY < 0 && maxY > -1 && (minX < 1 || minZ < 1);
            }
        };
        PhysicsResult result = new WalkingPhysics(input(false, false, false, true, false), 0)
            .step(world, new PhysicsState(new Vector3d(1.25, 0, 1.25),
                new Vector3d(0.2, 0, 0.2), true), PLAYER_HALF_WIDTH, 1.8);
        assertEquals(1.3, result.position().x, 1.0e-12);
        assertEquals(1.3, result.position().z, 1.0e-12);
        assertEquals(0.2 * GROUND_DRAG, result.velocity().x, 1.0e-12);
        assertEquals(0.2 * GROUND_DRAG, result.velocity().z, 1.0e-12);
    }

    @Test
    void sneakingUsesStepHeightAndSuppliedBodyWidth() {
        WalkingPhysics physics = new WalkingPhysics(input(false, false, false, true, false), 0);
        PhysicsState state = new PhysicsState(new Vector3d(1.15, 0, 0.5), new Vector3d(0.2, 0, 0), true);
        assertEquals(1.3, physics.step(supportWorld(0, 1, 0, 1, -0.5), state, PLAYER_HALF_WIDTH, 1.8).position().x, 1.0e-12);
        assertEquals(1.15, physics.step(supportWorld(0, 1, 0, 1, -0.61), state, PLAYER_HALF_WIDTH, 1.8).position().x, 1.0e-12);
        assertEquals(1.15, physics.step(supportWorld(0, 1, 0, 1, 0), state, 0.1, 1.8).position().x, 1.0e-12);
    }

    @Test
    void airborneAndUpwardSneakingDoNotClipEdges() {
        World world = supportWorld(0, 1, 0, 1, 0);
        WalkingPhysics physics = new WalkingPhysics(input(false, false, false, true, false), 0);
        for (boolean onGround : new boolean[]{true, false}) {
            // This overlap-only fixture has no downward resolver; start above
            // its floor to test sneaking rather than an already penetrating body.
            PhysicsResult result = physics.step(world, new PhysicsState(new Vector3d(1.25, 0.2, 0.5),
                new Vector3d(0.2, onGround ? 0.42 : -0.1, 0), onGround), PLAYER_HALF_WIDTH, 1.8);
            assertEquals(1.45, result.position().x, 1.0e-12);
        }
        PhysicsResult walking = new WalkingPhysics().step(world,
            new PhysicsState(new Vector3d(1.25, 0, 0.5), new Vector3d(0.2, 0, 0), true), PLAYER_HALF_WIDTH, 1.8);
        assertEquals(1.45, walking.position().x, 1.0e-12);
    }

    @Test
    void horizontalThresholdPreservesSmallComponentsOfMovingVector() {
        for (Vector3d velocity : new Vector3d[]{
                new Vector3d(0.0025, 0.002, -0.0025), new Vector3d(0.002, 0, 0.2),
                new Vector3d(-0.2, 0, -0.002)}) {
            PhysicsResult result = new WalkingPhysics().step(EMPTY_WORLD,
                state(velocity, false), PLAYER_HALF_WIDTH, 1.8);
            assertEquals(new Vector3d(velocity.x, 0, velocity.z), result.position());
        }
    }

    @Test
    void signedHorizontalSweepsClipToWallEvenWhenEndpointPassesThrough() {
        World wall = supportWorld(1, 1.1, 1, 1.1, 3);
        for (int axis : new int[]{0, 2}) {
            for (double sign : new double[]{-1, 1}) {
                Vector3d position = new Vector3d(1.05, 2, 1.05);
                position.setComponent(axis, sign > 0 ? 0.5 : 1.6);
                Vector3d velocity = new Vector3d().setComponent(axis, sign * 2);
                PhysicsResult result = new WalkingPhysics().step(wall,
                    new PhysicsState(position, velocity, false), PLAYER_HALF_WIDTH, 0.6);
                assertEquals(sign > 0 ? 1 - PLAYER_HALF_WIDTH : 1.1 + PLAYER_HALF_WIDTH, result.position().get(axis), 1.0e-12);
                assertEquals(0, result.velocity().get(axis));
                assertEquals(axis == 0, result.collidedX());
                assertEquals(axis == 2, result.collidedZ());
            }
        }
    }

    @Test
    void clippedWallContactDoesNotCancelNextUpwardTick() {
        for (int axis : new int[]{0, 2}) {
            World wall = axis == 0
                ? supportWorld(1, 2, 0, 1, 1)
                : supportWorld(0, 1, 1, 2, 1);
            WalkingPhysics physics = new WalkingPhysics();
            PhysicsResult contact = physics.step(wall, new PhysicsState(
                new Vector3d(0.5, 1.0 / 16.0, 0.5),
                new Vector3d(0, 0.42, 0).setComponent(axis, 0.2159), false), PLAYER_HALF_WIDTH, 1.8);
            Vector3d position = contact.position();
            assertEquals(1 - PLAYER_HALF_WIDTH, position.get(axis), 1.0e-12);
            assertFalse(wall.isBoxColliding(position.x - PLAYER_HALF_WIDTH, position.y, position.z - PLAYER_HALF_WIDTH,
                position.x + PLAYER_HALF_WIDTH, position.y + 1.8, position.z + PLAYER_HALF_WIDTH),
                "clipped contact must not round the resulting body into the wall");
            PhysicsResult rising = physics.step(wall,
                new PhysicsState(position, contact.velocity(), false), PLAYER_HALF_WIDTH, 1.8);
            assertEquals(position.y + contact.velocity().y, rising.position().y, 1.0e-12);
            assertTrue(rising.velocity().y > 0, "touching a wall must not act as a ceiling");
        }
    }

    @Test
    void largerHorizontalComponentMovesFirstAroundCorner() {
        World corner = supportWorld(1, 2, 1, 2, 3);
        PhysicsResult result = new WalkingPhysics().step(corner,
            new PhysicsState(new Vector3d(0.5, 2, 0.5), new Vector3d(0.3, 0, 0.4), false),
            PLAYER_HALF_WIDTH, 0.6);
        assertEquals(1 - PLAYER_HALF_WIDTH, result.position().x, 1.0e-12);
        assertEquals(0.9, result.position().z, 1.0e-12);
        assertTrue(result.collidedX());
        assertFalse(result.collidedZ());
    }

    @Test
    void upwardTravelClearsFullBlockBeforeHorizontalCollisionCheck() {
        World block = supportWorld(1, 2, 0, 1, 1);
        PhysicsState state = new PhysicsState(new Vector3d(0.5, 0.9, 0.5),
            new Vector3d(0.3, 0.2, 0), false);
        PhysicsResult result = new WalkingPhysics().step(block, state, PLAYER_HALF_WIDTH, 1.8);
        assertEquals(new Vector3d(0.8, 1.1, 0.5), result.position());
        assertFalse(result.collidedX());
        assertEquals(state.position(), new Vector3d(0.5, 0.9, 0.5));
    }

    private static World supportWorld(double left, double right, double near, double far, double top) {
        return new World() {
            @Override public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }
            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return minX < right && maxX > left && minZ < far && maxZ > near
                    && minY < top && maxY > top - 1;
            }
        };
    }

}
