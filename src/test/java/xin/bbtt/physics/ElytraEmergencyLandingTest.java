package xin.bbtt.physics;

import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class ElytraEmergencyLandingTest {
    private final ElytraFlightController controller = new ElytraFlightController();
    private static class Islands extends World {
        boolean loaded = true;
        boolean solid = true;
        double start = 20;
        @Override public boolean chunkLoaded(int x, int z) { return loaded; }
        @Override public Set<ChunkPosition> loadedChunks() {
            if (!loaded) return Set.of();
            var chunks = new java.util.HashSet<ChunkPosition>();
            for (int x = -10; x <= 10; x++)
                for (int z = -10; z <= 10; z++) chunks.add(new ChunkPosition(x, z));
            return chunks;
        }
        @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) {
            return solid && minY < 0 && maxZ >= start;
        }
        @Override public OptionalDouble findHighestCollisionTopExcluding(double minX, double minY,
                double minZ, double maxX, double maxY, double maxZ, Set<Vector3i> excluded) {
            return solid && minZ >= start && minY <= 0 && maxY >= 0
                    ? OptionalDouble.of(0) : OptionalDouble.empty();
        }
    }
    private static PhysicsState state(double height) {
        return new PhysicsState(new Vector3d(0.5, height, 0.5), new Vector3d(0, -0.1, 0.5), false);
    }
    @Test void loadedGroundBeyondOldRadiusIsSelectable() {
        var world = new Islands();
        world.start = 138;
        var target = ElytraFlightController.selectForcedLandingTarget(world, state(100), 0, false);
        assertTrue(target.z > 128 && target.z <= 160);
    }

    @Test void flatGroundDecisionCharacterization() {
        var world = new Islands();
        world.start = -1000;
        double[] heights = {.6, 8, 20};
        float[] pitches = {-10, -30, -55};
        for (int i = 0; i < heights.length; i++) {
            var initial = new PhysicsState(new Vector3d(.5, heights[i], .5), new Vector3d(0, -.4, .8), false);
            assertEquals(new ElytraFlightController.Decision(-180, pitches[i], false,
                    ElytraFlightController.Phase.FORCED_LANDING),
                    controller.forceLand(world, initial, 0, 0, false, 0, 16));
        }
    }

    @Test void steepDescentResetsFallDistanceBeforeTouchdown() {
        var world = new Islands();
        world.start = -1000;
        var state = new PhysicsState(new Vector3d(.5, 16, .5), new Vector3d(.21, -.8, .5), false, 50);
        float yaw = 0;
        float pitch = 10;
        for (int tick = 0; tick < 400 && !state.onGround(); tick++) {
            ElytraFlightController.Decision decision;
            try {
                decision = controller.forceLand(world, state, yaw, pitch, false, 0, 16);
            } catch (IllegalStateException unavailable) {
                decision = controller.emergencyHold(world, state, yaw, pitch, false);
            }
            yaw = decision.yaw();
            pitch = decision.pitch();
            var result = new GlidingPhysics(yaw, pitch, false).step(world, state, .299, .6);
            if (result.onGround()) {
                assertTrue(state.velocity().y > -.5 || state.fallDistance() < 3);
            }
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance());
        }
        assertTrue(state.onGround());
    }

    @Test void prefersNearestGlidableSolidIslandWithoutSpendingRocket() {
        var world = new Islands();
        assertEquals(new Vector3d(0.5, 0, 24.5), controller.selectForcedLandingTarget(world, state(20), 8, false));
        var decision = controller.forceLand(world, state(20), 0, 0, false, 8, 16);
        assertEquals(ElytraFlightController.Phase.FORCED_LANDING, decision.phase());
        assertFalse(decision.firework());
    }
    @Test void usesRocketToReachGroundOutsideGlideEnvelope() {
        var world = new Islands();
        world.start = 40;
        assertThrows(IllegalStateException.class,
                () -> controller.selectForcedLandingTarget(world, state(3), 0, false));
        assertTrue(controller.forceLand(world, state(3), 0, 0, false, 1, 16).firework());
        assertFalse(controller.forceLand(world, state(3), 0, 0, true, 1, 16).firework());
    }
    @Test void lowGlideAcceptsSafeTouchdownBeyondNearestSiteCentre() {
        var world = new Islands();
        world.start = -1000;
        PhysicsState state = new PhysicsState(new Vector3d(.5, .6, .5), new Vector3d(0, -.4, .8), false);
        float yaw = 0;
        float pitch = 0;
        for (int tick = 0; tick < 100 && !state.onGround(); tick++) {
            var decision = controller.forceLand(world, state, yaw, pitch, false, 0, 16);
            assertFalse(decision.firework());
            yaw = decision.yaw();
            pitch = decision.pitch();
            var result = new GlidingPhysics(yaw, pitch, false).step(world, state, .299, .6);
            assertFalse(result.collidedX() || result.collidedZ());
            state = new PhysicsState(result.position(), result.velocity(), result.onGround());
        }
        assertTrue(state.onGround());
        assertTrue(state.position().z > 1.5, "safe landing need not be within one block of the original centre");
    }

    @Test void emergencyApproachRetainsHeightUntilIslandIsReachable() {
        var world = new Islands();
        var initial = state(8);
        var decision = controller.forceLand(world, initial, 0, 10, false, 0, 16);
        var target = controller.selectForcedLandingTarget(world, initial, 0, false);
        var simulated = initial;
        for (int tick = 0; tick < 24; tick++) {
            var result = new GlidingPhysics(decision.yaw(), decision.pitch(), false)
                    .step(world, simulated, .299, .6);
            if (result.position().z < world.start + 2) {
                assertTrue(result.position().y >= Math.hypot(result.position().x - target.x,
                        result.position().z - target.z) / 5);
            }
            simulated = new PhysicsState(result.position(), result.velocity(), result.onGround());
        }
    }

    @Test void unknownTerrainHoldDoesNotInventSafeLandingOrSpendRocket() {
        var world = new Islands();
        world.loaded = false;
        var decision = controller.emergencyHold(world, state(20), 37, 80, false);
        assertEquals(ElytraFlightController.Phase.FORCED_LANDING, decision.phase());
        assertEquals(37, decision.yaw());
        assertEquals(-10, decision.pitch());
        assertFalse(decision.firework());
    }

    @Test void voidAndUnknownColumnsNeverBecomeWorldBottomLandingSites() {
        var world = new Islands();
        world.solid = false;
        var error = assertThrows(IllegalStateException.class,
                () -> controller.forceLand(world, state(20), 0, 0, false, 64, 16));
        assertTrue(error.getMessage().contains("No known solid"));
        world.solid = true;
        world.loaded = false;
        assertThrows(IllegalStateException.class,
                () -> controller.selectForcedLandingTarget(world, state(20), 64, true));
    }
}
