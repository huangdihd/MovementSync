package xin.bbtt.physics;

import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

class LocalLookaheadSolverTest {
    private final LocalLookaheadSolver solver = new LocalLookaheadSolver();
    private final World air = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
    };

    private static PhysicsState state(double x, double vx) {
        return new PhysicsState(new Vector3d(x, 100, 8), new Vector3d(vx, 0, 0),
                false, 0, PlayerPose.GLIDING);
    }

    private LocalLookaheadSolver.Result solve(World world, PhysicsState state, int steps, int horizon) {
        return solver.solve(List.of(new Vector3d(-40, 100, 8)), state, world,
                MovementParameters.DEFAULT, 90, 0, false,
                new LocalLookaheadSolver.Budget(steps, horizon, 0));
    }

    @Test void syntheticReportedRouteNeverSelectsReverseYaw() {
        var start = new Vector3d(-3.5, 36, -1.5);
        var goal = new Vector3d(175, 60, -32);
        var route = new LocalAirRoute(new WaypointFlightTarget(List.of(start, goal), goal, false));
        var direction = new Vector3d(goal).sub(start).normalize();
        var current = new PhysicsState(start, new Vector3d(direction).mul(.8), false, 0, PlayerPose.GLIDING);
        float reverse = LowAltitudePlanner.yaw(new Vector3d(direction).negate());
        for (int i = 0; i < 12; i++) {
            var points = route.lookahead(air, current);
            assertFalse(points.isEmpty());
            for (var point : points) assertTrue(new Vector3d(point).sub(current.position()).dot(direction) > 0);
            var result = solver.solve(points, current, air, MovementParameters.DEFAULT, reverse, 0, false,
                    new LocalLookaheadSolver.Budget(1000, 20, 0));
            assertNotNull(result.decision());
            double radians = Math.toRadians(result.decision().yaw());
            assertTrue(new Vector3d(-Math.sin(radians), 0, Math.cos(radians)).dot(direction) > 0);
            current = new PhysicsState(new Vector3d(start).lerp(goal, (i + 1) / 15.0),
                    new Vector3d(direction).mul(.8), false, 0, PlayerPose.GLIDING);
        }
    }

    @Test void zeroBudgetDoesNotProduceAnUnsimulatedDecision() {
        var result = solve(air, state(8, -1), 0, 4);
        assertEquals(LocalLookaheadSolver.Status.BUDGET_EXHAUSTED, result.status());
        assertEquals(0, result.steps());
        assertNull(result.decision());
        assertNull(result.predictedState());
        assertTrue(result.elapsedNanos() >= 0);
    }

    @Test void partialCandidateCannotWinAndAllCandidatesShareOneBudget() {
        var partial = solve(air, state(8, -1), 3, 4);
        assertEquals(3, partial.steps());
        assertNull(partial.decision());
        var complete = solve(air, state(8, -1), 7, 4);
        assertEquals(7, complete.steps());
        assertEquals(LocalLookaheadSolver.Status.BUDGET_EXHAUSTED, complete.status());
        assertNotNull(complete.decision());
        assertFalse(complete.decision().firework());
        assertNotNull(complete.predictedState());
    }

    @Test void winningPredictionUsesExactGlidingPhysics() {
        var initial = state(8, -1);
        var result = solve(air, initial, 1000, 5);
        assertEquals(LocalLookaheadSolver.Status.COMPLETE, result.status());
        var control = result.decision();
        var physics = new GlidingPhysics(control.yaw(), control.pitch(), false,
                PlayerInput.NONE, MovementParameters.DEFAULT);
        var predicted = initial;
        for (int i = 0; i < 5; i++) {
            var step = physics.step(air, predicted, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            predicted = new PhysicsState(step.position(), step.velocity(), step.onGround(),
                    step.fallDistance(), PlayerPose.GLIDING);
        }
        assertEquals(predicted, result.predictedState());
        assertTrue(predicted.position().x < initial.position().x);
    }

    @Test void negativeSweepRejectsFireEvenWhenBothEndpointsAreClear() {
        World fire = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return (int) Math.floor(position.x) == -2
                        ? new BlockState("fire", 0, Map.of(), "empty", 0, false, "")
                        : super.getBlockStateAt(position);
            }
        };
        var initial = state(1, -6);
        var raw = new GlidingPhysics(90, 0, false).step(fire, initial,
                PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        assertTrue(raw.position().x < -3);
        assertFalse(raw.collidedX());
        var result = solve(fire, initial, 1000, 1);
        assertEquals(LocalLookaheadSolver.Status.NO_SAFE_CANDIDATE, result.status());
        assertNull(result.decision());
    }

    @Test void negativeSweepRejectsUnloadedIntermediateChunk() {
        World gap = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return x != -1; }
        };
        var result = solve(gap, state(8, -50), 1000, 1);
        assertEquals(LocalLookaheadSolver.Status.NO_SAFE_CANDIDATE, result.status());
        assertNull(result.decision());
    }

    @Test void movingTargetCopiesVectorsAndScoresFuturePositionAndVelocity() {
        Vector3d position = new Vector3d(10, 100, 8);
        Vector3d velocity = new Vector3d(2, 0, 0);
        var target = new LocalLookaheadSolver.MovingTarget(position, velocity);
        position.zero();
        velocity.zero();
        target.position().zero();
        target.velocity().zero();
        target.predict(5).zero();
        assertEquals(new Vector3d(20, 100, 8), target.predict(5));
        var intercept = new PhysicsState(new Vector3d(20, 100, 8), new Vector3d(2, 0, 0), false);
        var oldPosition = new PhysicsState(new Vector3d(10, 100, 8), new Vector3d(2, 0, 0), false);
        var wrongVelocity = new PhysicsState(intercept.position(), new Vector3d(-2, 0, 0), false);
        assertEquals(0, target.score(intercept, 5), 0.0);
        assertTrue(target.score(intercept, 5) > target.score(oldPosition, 5));
        assertTrue(target.score(intercept, 5) > target.score(wrongVelocity, 5));
        assertThrows(IllegalArgumentException.class, () -> target.predict(-1));
    }

    @Test void attachedRocketPredictionDoesNotRequestAnotherRocket() {
        var initial = state(8, -1);
        var result = solver.solve(List.of(new Vector3d(-40, 100, 8)), initial, air,
                MovementParameters.DEFAULT, 90, 0, true, new LocalLookaheadSolver.Budget(1, 1, 0));
        assertNotNull(result.decision());
        assertFalse(result.decision().firework());
        var expected = new GlidingPhysics(result.decision().yaw(), result.decision().pitch(), true,
                PlayerInput.NONE, MovementParameters.DEFAULT).step(air, initial,
                PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        assertEquals(expected.position(), result.predictedState().position());
        assertEquals(expected.velocity(), result.predictedState().velocity());
        assertEquals(1, result.steps());
    }

    @Test void deadlineIsSharedAndDoesNotAcceptPartialPreview() {
        var result = solver.solve(List.of(new Vector3d(-40, 100, 8)), state(8, -1), air,
                MovementParameters.DEFAULT, 90, 0, false, new LocalLookaheadSolver.Budget(1000, 20, 1));
        assertEquals(LocalLookaheadSolver.Status.BUDGET_EXHAUSTED, result.status());
        assertNull(result.decision());
        assertTrue(result.steps() < 20);
    }
}
