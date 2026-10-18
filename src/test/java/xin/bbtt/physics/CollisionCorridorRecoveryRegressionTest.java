package xin.bbtt.physics;

import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Synthetic loaded geometry, not a reconstruction of unobserved live terrain. */
final class CollisionCorridorRecoveryRegressionTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0, Map.of(), "empty", 0, false, "");

    private World complexWall() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                // West face, raised target shelf, east buttress and overhead lip.
                if (p.y < 32 || p.x >= 160 && p.x < 164 && p.z > -28 && p.z < 4 && p.y < 68
                        || p.x >= 170 && p.x < 185 && p.z >= -40 && p.z < -28 && p.y < 60
                        || p.x >= 190 && p.x < 195 && p.z > -40 && p.y < 74
                        || p.x >= 164 && p.x < 180 && p.z > -20 && p.z < 0 && p.y >= 68 && p.y < 70)
                    return STONE;
                return super.getBlockStateAt(p);
            }
        };
    }

    @Test void zeroDamageWallClippingIsNotAnAcceptedGeometricTrajectory() {
        var world = complexWall();
        var initial = new PhysicsState(new Vector3d(159.69, 55.63, -17.91),
                new Vector3d(.12, .312, .1197), false, 0, PlayerPose.GLIDING);
        var route = new WaypointFlightTarget(List.of(initial.position(), new Vector3d(175, 60, -32)),
                new Vector3d(175, 60, -32), false, false, 30);
        var trajectory = FlightTrajectory.simulate(world, initial,
                new GlidingPhysics(274.24f, -10, false), 6,
                (before, result) -> result.collidedX() || result.collidedZ() || result.onGround());
        assertTrue(trajectory.result(0).collidedX(), "Actual physics must reproduce the west-face clip");
        assertTrue(route.contains(initial.position(), trajectory.result(0).position()),
                "Geometric proximity alone used to admit this clipped step");
        assertFalse(route.accepts(trajectory));
    }

    @Test void terminalBrakingNearComplexWallNeverEscapesItsProvenCorridor() {
        var world = complexWall();
        var initial = new PhysicsState(new Vector3d(158, 55, -18), new Vector3d(.1, -.1, -.1),
                false, 0, PlayerPose.GLIDING);
        var goal = new Vector3d(175, 60, -32);
        var route = new WaypointFlightTarget(List.of(initial.position(), new Vector3d(158, 57, -32), goal),
                goal, true, false, 3, world);
        try {
            var decision = new ElytraFlightController().decide(world, initial, 274.24f, -10, false, route, 2, 2);
            var trajectory = FlightTrajectory.simulate(world, initial,
                    new GlidingPhysics(decision.yaw(), decision.pitch(), decision.firework()), 6,
                    (before, result) -> result.onGround() || result.collidedX() || result.collidedZ());
            assertTrue(route.accepts(trajectory));
            for (int tick = 0; tick < trajectory.size(); tick++) {
                assertFalse(trajectory.result(tick).collidedX());
                assertFalse(trajectory.result(tick).collidedZ());
            }
        } catch (IllegalStateException noSafeTrajectory) {
            assertTrue(noSafeTrajectory.getMessage().contains("No safe low-altitude trajectory"),
                    "Rejecting an unprovable corridor is preferable to clipping the wall");
        }
    }

    private World island() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y < 32) return Math.hypot(p.x, p.z) < 6 ? LAVA : STONE;
                return super.getBlockStateAt(p);
            }
        };
    }

    @Test void targetPreferenceRetainsSafeFallbackAndRespectsCooldownAndRuleVeto() {
        var world = island();
        var state = new PhysicsState(new Vector3d(0, 40, 0), new Vector3d(.4, 0, 0),
                false, 0, PlayerPose.GLIDING);
        var resources = new HazardRecovery.Context(new int[] {1, 1}, 0, false, null);
        var fallback = HazardRecovery.assess(world, state, MovementParameters.DEFAULT, resources);
        assertEquals(HazardRecovery.Status.RECOVERABLE, fallback.status());
        var goal = new Vector3d(-20, 32, 0);
        var preferred = HazardRecovery.assessToward(world, state, MovementParameters.DEFAULT, resources, goal, d -> true);
        assertEquals(HazardRecovery.Status.RECOVERABLE, preferred.status());
        assertTrue(preferred.target().distance(goal) <= fallback.target().distance(goal));
        var vetoed = HazardRecovery.assessToward(world, state, MovementParameters.DEFAULT, resources, goal, d -> false);
        assertEquals(fallback.target(), vetoed.target(), "A rejected improvement must not erase the safe fallback");
        var cooldown = new HazardRecovery.Context(new int[] {1, 1}, 20, false, null);
        var waiting = HazardRecovery.assessToward(world, state, MovementParameters.DEFAULT, cooldown, goal,
                d -> { fail("No preference search while rockets are unavailable now"); return true; });
        assertEquals(HazardRecovery.Status.RECOVERABLE, waiting.status());
        assertFalse(waiting.decision().firework());
        var unknown = HazardRecovery.assessToward(new World(), state, MovementParameters.DEFAULT, resources, goal, d -> true);
        assertEquals(HazardRecovery.Status.UNPROVEN, unknown.status());
    }
}
