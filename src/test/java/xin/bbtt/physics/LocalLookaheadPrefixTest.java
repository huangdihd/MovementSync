package xin.bbtt.physics;

import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

class LocalLookaheadPrefixTest {
    private final LocalLookaheadSolver solver = new LocalLookaheadSolver();
    private final World air = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
    };
    private final PhysicsState deployment = new PhysicsState(new Vector3d(55, 45, -11.5),
            new Vector3d(1.16, -.10, -.23), false, 0, PlayerPose.GLIDING);
    private final List<Vector3d> route = List.of(new Vector3d(175, 60, -32));

    private LocalLookaheadSolver.PrefixResult solve(World world, int steps, long nanos) {
        return solver.solvePrefix(route, deployment, world, MovementParameters.DEFAULT,
                90, 0, false, new LocalLookaheadSolver.Budget(steps, 20, nanos));
    }

    private void assertForward(LocalLookaheadSolver.PrefixResult result) {
        double yaw = Math.toRadians(result.decision().yaw());
        assertTrue(new Vector3d(-Math.sin(yaw), 0, Math.cos(yaw))
                .dot(new Vector3d(route.get(0)).sub(deployment.position())) > 0);
        assertFalse(result.decision().firework());
    }

    @Test void deploymentShortPrefixDoesNotRequireTwentyTicksAndReplaysExactly() {
        var result = solve(air, 3, 0);
        assertEquals(LocalLookaheadSolver.PrefixStatus.SAFE_PREFIX, result.status());
        assertEquals(3, result.prefix().size());
        assertEquals(3, result.steps());
        assertEquals(1, result.rejections().budget());
        assertForward(result);
        var state = deployment;
        for (var step : result.prefix()) {
            assertEquals(state, step.expected());
            assertEquals(result.decision(), step.control());
            var raw = new GlidingPhysics(step.control().yaw(), step.control().pitch(), false,
                    PlayerInput.NONE, MovementParameters.DEFAULT)
                    .step(air, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            state = new PhysicsState(raw.position(), raw.velocity(), raw.onGround(),
                    raw.fallDistance(), PlayerPose.GLIDING);
        }
        assertEquals(state, result.predictedState());
        assertThrows(UnsupportedOperationException.class, () -> result.prefix().clear());
    }

    @Test void deploymentNeverSelectsReverseEvenWithFullSearch() {
        var result = solve(air, 1000, 0);
        assertEquals(LocalLookaheadSolver.PrefixStatus.SAFE_PREFIX, result.status());
        assertEquals(20, result.prefix().size());
        assertForward(result);
    }

    @Test void verticalCorridorEvaluatesForwardTangentBeforeBudgetExhaustion() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(),
                false, 0, PlayerPose.GLIDING);
        var goal = new Vector3d(20, 120, 0);
        var corridor = new WaypointFlightTarget(List.of(initial.position(),
                new Vector3d(0, 120, 0), goal), goal, false);

        var result = solver.solvePrefix(List.of(new Vector3d(.01, 120, 0)), initial,
                air, MovementParameters.DEFAULT, 0, 0, false,
                new LocalLookaheadSolver.Budget(1, 1, 0), null, corridor);

        assertEquals(LocalLookaheadSolver.PrefixStatus.SAFE_PREFIX, result.status());
        assertEquals(1, result.prefix().size());
        assertEquals(-90, result.decision().yaw());
        assertEquals(1, result.rejections().budget());
    }

    @Test void plannerCorridorPrioritizesForwardTangentBeforeBudgetExhaustion() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(),
                false, 0, PlayerPose.GLIDING);
        var goal = new Vector3d(20, 100, 20);
        var corridor = new WaypointFlightTarget(List.of(initial.position(), goal), goal, false);

        var result = solver.solvePrefix(List.of(new Vector3d(20 * Math.sqrt(3), 100, 20)), initial,
                air, MovementParameters.DEFAULT, 0, 0, false,
                new LocalLookaheadSolver.Budget(1, 1, 0), null, corridor);

        assertEquals(LocalLookaheadSolver.PrefixStatus.SAFE_PREFIX, result.status());
        assertEquals(-45, result.decision().yaw());
        assertEquals(1, result.rejections().budget());
    }

    @Test void stalePlannerTangentCannotReintroduceAReverseYaw() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(),
                false, 0, PlayerPose.GLIDING);
        var staleGoal = new Vector3d(20, 100, 0);
        var corridor = new WaypointFlightTarget(List.of(initial.position(), staleGoal),
                staleGoal, false, false, 100);
        var liveAim = new Vector3d(-20, 100, 0);

        var result = solver.solvePrefix(List.of(liveAim), initial, air,
                MovementParameters.DEFAULT, -90, 0, false,
                new LocalLookaheadSolver.Budget(1, 1, 0), null, corridor);

        assertEquals(LocalLookaheadSolver.PrefixStatus.SAFE_PREFIX, result.status());
        double radians = Math.toRadians(result.decision().yaw());
        assertTrue(new Vector3d(-Math.sin(radians), 0, Math.cos(radians))
                .dot(new Vector3d(liveAim).sub(initial.position())) > 0);
        assertEquals(90, result.decision().yaw());
        assertEquals(1, result.rejections().budget());
    }

    @Test void farthestFirstRouteOrderCannotBeReversedByANearPassedSample() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(),
                false, 0, PlayerPose.GLIDING);
        var forward = new Vector3d(0, 100, 20);
        var passed = new Vector3d(0, 100, -1);

        var result = solver.solvePrefix(List.of(forward, passed), initial, air,
                MovementParameters.DEFAULT, 0, 0, false,
                new LocalLookaheadSolver.Budget(1, 1, 0));

        assertEquals(LocalLookaheadSolver.PrefixStatus.SAFE_PREFIX, result.status());
        double radians = Math.toRadians(result.decision().yaw());
        assertTrue(new Vector3d(-Math.sin(radians), 0, Math.cos(radians))
                        .dot(new Vector3d(forward).sub(initial.position())) > 0,
                "solvePrefix must preserve LocalAirRoute's farthest-first forward hemisphere");
        assertEquals(0, result.decision().yaw());
        assertEquals(1, result.rejections().budget());
    }

    @Test void validatorSkipsTheHighestRankedCandidateAndSelectsTheNextSafeOne() {
        var budget = new LocalLookaheadSolver.Budget(1000, 20, 0);
        var baseline = solver.solvePrefix(route, deployment, air, MovementParameters.DEFAULT,
                90, 0, true, budget, null, null);
        var rejected = baseline.decision();

        var result = solver.solvePrefix(route, deployment, air, MovementParameters.DEFAULT,
                90, 0, true, budget, null, null,
                prefix -> prefix.get(0).control().equals(rejected) ? 0 : prefix.size());

        assertEquals(LocalLookaheadSolver.PrefixStatus.SAFE_PREFIX, result.status());
        assertEquals(20, result.prefix().size());
        assertNotEquals(rejected, result.decision());
    }

    @Test void validatorRetainsTheLongestProvenPartOfACandidate() {
        var result = solver.solvePrefix(route, deployment, air, MovementParameters.DEFAULT,
                90, 0, true, new LocalLookaheadSolver.Budget(1000, 20, 0), null, null,
                prefix -> Math.min(7, prefix.size()));

        assertEquals(LocalLookaheadSolver.PrefixStatus.SAFE_PREFIX, result.status());
        assertEquals(7, result.prefix().size());
        assertEquals(result.prefix().get(0).control(), result.decision());
    }

    @Test void validatorCannotBeBypassedWhenEveryCandidateIsUnsafe() {
        var result = solver.solvePrefix(route, deployment, air, MovementParameters.DEFAULT,
                90, 0, true, new LocalLookaheadSolver.Budget(1000, 20, 0), null, null,
                prefix -> 0);

        assertEquals(LocalLookaheadSolver.PrefixStatus.UNPROVEN, result.status());
        assertTrue(result.prefix().isEmpty());
        assertNull(result.predictedState());
    }

    @Test void plannerBackedPursuitNeverLetsLiveTargetAltitudeOverrideTheCorridor() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(.4, 0, 0),
                false, 0, PlayerPose.GLIDING);
        var goal = new Vector3d(40, 100, 0);
        var corridor = new WaypointFlightTarget(List.of(initial.position(), goal), goal, false,
                false, 20);
        var lowTarget = new LocalLookaheadSolver.MovingTarget(
                new Vector3d(40, 64, 0), new Vector3d(.8, 0, 0));
        var highTarget = new LocalLookaheadSolver.MovingTarget(
                new Vector3d(40, 300, 0), new Vector3d(.8, 10, 0));
        var budget = new LocalLookaheadSolver.Budget(1000, 12, 0);

        var low = solver.solvePrefix(List.of(goal), initial, air, MovementParameters.DEFAULT,
                -90, 0, false, budget, lowTarget, corridor);
        var high = solver.solvePrefix(List.of(goal), initial, air, MovementParameters.DEFAULT,
                -90, 0, false, budget, highTarget, corridor);

        assertEquals(LocalLookaheadSolver.PrefixStatus.SAFE_PREFIX, low.status());
        assertEquals(low.decision(), high.decision());
        assertEquals(low.score(), high.score(), 0.0);
    }

    @Test void shortSteeringSamplesDoNotPenalizePoweredProgressAlongRemainingRoute() {
        var initial = new PhysicsState(new Vector3d(16.5, 65.56, 282.8),
                new Vector3d(0, .007474, .7228), false, 1, PlayerPose.GLIDING);
        var anchors = List.of(new Vector3d(16.5, 65.56, 291.3),
                new Vector3d(16.5, 65.56, 287.3));
        var progress = List.of(new Vector3d(16.5, 65.56, 333.4));
        var budget = new LocalLookaheadSolver.Budget(1000, 12, 0);
        var unpowered = solver.solvePrefix(anchors, initial, air, MovementParameters.DEFAULT,
                0, -20, false, budget, null, null, null, progress);
        var powered = solver.solvePrefix(anchors, initial, air, MovementParameters.DEFAULT,
                0, -20, true, budget, null, null, null, progress);

        assertEquals(12, unpowered.prefix().size());
        assertEquals(12, powered.prefix().size());
        assertTrue(powered.predictedState().position().z > anchors.get(0).z);
        assertTrue(powered.predictedState().position().z > unpowered.predictedState().position().z);
        assertTrue(powered.score() > unpowered.score(),
                "passing a steering sample is forward progress, not endpoint overshoot");
        assertEquals(0, powered.decision().yaw());
        var localUnpowered = solver.solvePrefix(anchors, initial, air, MovementParameters.DEFAULT,
                0, -20, false, budget);
        var localPowered = solver.solvePrefix(anchors, initial, air, MovementParameters.DEFAULT,
                0, -20, true, budget);
        assertTrue(localPowered.score() < localUnpowered.score(),
                "the short-sample metric must reproduce the original false preference");
        assertEquals(localUnpowered.decision(), unpowered.decision());
        assertEquals(localPowered.decision(), powered.decision(),
                "progress comparison must not replace the local steering selection");
    }

    @Test void longestPrefixWinsOverPartiallyEvaluatedChallenger() {
        var result = solve(air, 23, 0);
        assertEquals(20, result.prefix().size());
        assertEquals(23, result.steps());
        assertEquals(1, result.rejections().budget());
    }

    @Test void zeroBudgetAndDeadlineAreExplicitlyUnprovenForwardFallbacks() {
        for (var result : List.of(solve(air, 0, 0), solve(air, 1000, 1))) {
            assertEquals(LocalLookaheadSolver.PrefixStatus.UNPROVEN, result.status());
            assertTrue(result.prefix().isEmpty());
            assertNull(result.predictedState());
            assertEquals(-20, result.decision().pitch());
            assertEquals(1, result.rejections().budget());
            assertForward(result);
        }
    }

    @Test void unknownCollisionAndDamageAreCountedSeparately() {
        World unknown = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
        };
        World collision = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) { return true; }
        };
        World damage = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return new BlockState("fire", 0, Map.of(), "empty", 0, false, "");
            }
        };
        var u = solve(unknown, 1000, 0);
        var c = solve(collision, 1000, 0);
        var d = solve(damage, 1000, 0);
        assertTrue(u.rejections().unknown() > 0);
        assertEquals(0, u.rejections().collision());
        assertEquals(0, u.rejections().damage());
        assertTrue(c.rejections().collision() > 0);
        assertEquals(0, c.rejections().unknown());
        assertEquals(0, c.rejections().damage());
        assertTrue(d.rejections().damage() > 0);
        assertEquals(0, d.rejections().unknown());
        assertEquals(0, d.rejections().collision());
        for (var result : List.of(u, c, d)) {
            assertEquals(LocalLookaheadSolver.PrefixStatus.UNPROVEN, result.status());
            assertTrue(result.prefix().isEmpty());
            assertEquals(-20, result.decision().pitch());
            assertForward(result);
        }
    }

    @Test void corridorDeparturesAreCountedSeparately() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .3),
                false, 0, PlayerPose.GLIDING);
        var corridor = new WaypointFlightTarget(List.of(initial.position()), initial.position(),
                false, false, 0);

        var result = solver.solvePrefix(List.of(new Vector3d(0, 100, 20)), initial, air,
                MovementParameters.DEFAULT, 0, 0, false,
                new LocalLookaheadSolver.Budget(1000, 1, 0), null, corridor);

        assertEquals(LocalLookaheadSolver.PrefixStatus.UNPROVEN, result.status());
        assertTrue(result.rejections().corridor() > 0);
        assertEquals(result.steps(), result.rejections().corridor());
        assertEquals(0, result.rejections().unknown());
        assertEquals(0, result.rejections().collision());
        assertEquals(0, result.rejections().damage());
        assertEquals(0, result.rejections().budget());
    }

    @Test void safePrefixBeforeUnknownBoundaryIsRetained() {
        World boundary = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return x < 4; }
        };
        var result = solve(boundary, 1000, 0);
        assertEquals(LocalLookaheadSolver.PrefixStatus.SAFE_PREFIX, result.status());
        assertFalse(result.prefix().isEmpty());
        assertTrue(result.prefix().size() < 20);
        assertTrue(result.rejections().unknown() > 0);
        assertForward(result);
    }
}
