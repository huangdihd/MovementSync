package xin.bbtt.physics;

import java.util.Arrays;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class KnownReturnYawReplayTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1,
            Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0,
            Map.of("level", "0"), "empty", 0, false, "LAVA");
    private static final Vector3d ANCHOR = new Vector3d(-8.5, 40, .5);
    private final World world = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d point) {
            if (point.y >= 120 || point.x < 0 && point.y < 40) return STONE;
            if (point.y < 31) return LAVA;
            return super.getBlockStateAt(point);
        }
    };

    /** Full-precision state captured at tick 1438 of the absent-far-shore physical test. */
    private PhysicsState liveState() {
        return new PhysicsState(new Vector3d(247.73972122269112, 47.521826053022316, .40177325394618090),
                new Vector3d(-1.4004069286290580, .78451491878159880, .00051236007623334670),
                false, 1, PlayerPose.GLIDING);
    }

    private HazardRecovery.Context liveResources() {
        int[] flights = new int[29];
        Arrays.fill(flights, 1);
        return new HazardRecovery.Context(flights, 38, true, null, 19);
    }

    @Test void equivalentHeadingsCanExceedThePhysicalCheckpointTolerance() {
        var initial = liveState();
        var delta = new Vector3d(ANCHOR).sub(initial.position());
        float normalized = (float) Math.toDegrees(Math.atan2(-delta.x, delta.z));
        float continuous = MovementSync.nearestEquivalentYaw(-1350.02197f, normalized);
        var predicted = new GlidingPhysics(normalized, -30, true, PlayerInput.NONE, MovementParameters.DEFAULT)
                .step(world, initial, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        var executed = new GlidingPhysics(continuous, -30, true, PlayerInput.NONE, MovementParameters.DEFAULT)
                .step(world, initial, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        assertTrue(predicted.position().distance(executed.position()) > 1.0e-5);
        assertTrue(predicted.velocity().distance(executed.velocity()) > 1.0e-5);
    }

    @ParameterizedTest
    @ValueSource(floats = {-1350.02197f, -3149.889f, 3689.978f})
    void accumulatedYawReplaysEveryProvenStateExactly(float initialYaw) {
        var initial = liveState();
        var plan = KnownReturnPlan.preview(world, initial, MovementParameters.DEFAULT,
                liveResources(), ANCHOR, 0, initialYaw);
        assertExecutable(initial, initialYaw, plan);
    }

    @Test void latePoweredDescentRetainsItsProvenFlare() {
        // Rounded tick-1596 log coordinates deliberately form an independent fixed fixture.
        var initial = new PhysicsState(new Vector3d(9.931, 58.70, .5356),
                new Vector3d(-1.686, -.01581, -.003395), false,
                1.0316298896463323, PlayerPose.GLIDING);
        var resources = new HazardRecovery.Context(new int[0], 22, true, null, 3);
        float initialYaw = -3149.889f;
        var plan = KnownReturnPlan.preview(world, initial, MovementParameters.DEFAULT,
                resources, ANCHOR, 0, initialYaw);
        assertExecutable(initial, initialYaw, plan);
        assertEquals(0, plan.requiredRockets());
        assertTrue(plan.steps().get(0).control().pitch() > 0);
        assertEquals(-35, plan.steps().get(1).control().pitch());
    }

    @Test void exactFailingLiveDescentRetainsItsProvenFlare() {
        // Full-precision tick 1621 from a failing absent-far-shore run. The first
        // physical mismatch revoked the witness before its tick-1622 flare.
        var initial = new PhysicsState(new Vector3d(5.8619256461595315, 56.472486887244266, .65994398385115540),
                new Vector3d(-1.5355583437715874, .13278929378380190, -.016824367353092836),
                false, 1, PlayerPose.GLIDING);
        int[] flights = new int[24];
        Arrays.fill(flights, 1);
        var resources = new HazardRecovery.Context(flights, 37, true, null, 18);
        float initialYaw = -4949.36377f;
        var plan = KnownReturnPlan.preview(world, initial, MovementParameters.DEFAULT,
                resources, ANCHOR, 0, initialYaw);
        assertExecutable(initial, initialYaw, plan);
        assertEquals(0, plan.requiredRockets());
        assertTrue(plan.steps().get(0).control().pitch() > 0);
        assertEquals(-35, plan.steps().get(1).control().pitch());
    }

    @Test void fixedPolicyReplayRebasesYawBeforePredicting() {
        var initial = liveState();
        var normalized = KnownReturnPlan.preview(world, initial, MovementParameters.DEFAULT,
                liveResources(), ANCHOR, 0);
        assertTrue(normalized.proven());
        float initialYaw = -1350.02197f;
        var replay = KnownReturnPlan.replay(world, initial, MovementParameters.DEFAULT, normalized, initialYaw);
        assertExecutable(initial, initialYaw, replay);
        for (int index = 0; index < replay.steps().size(); index++) {
            var original = normalized.steps().get(index);
            var rebased = replay.steps().get(index);
            assertEquals(original.control().pitch(), rebased.control().pitch());
            assertEquals(original.control().firework(), rebased.control().firework());
            assertEquals(original.control().phase(), rebased.control().phase());
            assertEquals(original.resource(), rebased.resource());
        }
    }

    @ParameterizedTest
    @ValueSource(floats = {Float.NaN, Float.POSITIVE_INFINITY, Float.NEGATIVE_INFINITY})
    void nonFiniteYawCannotCertifyAPlan(float initialYaw) {
        var result = KnownReturnPlan.preview(world, liveState(), MovementParameters.DEFAULT,
                liveResources(), ANCHOR, 0, initialYaw);
        assertFalse(result.proven());
        assertEquals(0, result.simulatedSteps());
    }

    private void assertExecutable(PhysicsState initial, float initialYaw, KnownReturnPlan.Result plan) {
        assertTrue(plan.proven(), plan.reason());
        var actual = initial;
        float yaw = initialYaw;
        for (int index = 0; index < plan.steps().size(); index++) {
            var expected = plan.steps().get(index);
            assertEquals(expected.before(), actual, "checkpoint " + index);
            float previousYaw = yaw;
            // This is the exact transformation used by live MovementSync.setYawToward.
            yaw = MovementSync.nearestEquivalentYaw(yaw, expected.control().yaw());
            var next = new GlidingPhysics(yaw, expected.control().pitch(), expected.resource().powered(),
                    PlayerInput.NONE, MovementParameters.DEFAULT)
                    .step(world, actual, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            actual = FlightSafety.nextState(next);
            assertEquals(expected.after(), actual, "physical replay " + index);
            assertEquals(expected.control().yaw(), yaw, "stored execution yaw " + index);
            assertTrue(Math.abs((double) yaw - previousYaw) <= 180, "continuous yaw " + index);
            assertTrue(next.fallDistance() < 3, "strict fall margin " + index);
            assertFalse(next.collidedX() || next.collidedZ(), "horizontal impact " + index);
        }
        assertTrue(actual.onGround());
        assertTrue(actual.position().x < 0);
        assertEquals(40, actual.position().y);
        assertFalse(plan.steps().get(plan.steps().size() - 1).resource().powered());
    }
}
