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
