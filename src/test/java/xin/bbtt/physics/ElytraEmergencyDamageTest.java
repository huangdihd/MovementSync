package xin.bbtt.physics;

import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class ElytraEmergencyDamageTest {
    private final ElytraFlightController controller = new ElytraFlightController();
    private static final float[] PITCHES = {-55, -30, -10, 0, 10};
    private static final float[] YAW_OFFSETS = {0, -30, 30, -60, 60, -100, 100, 180};

    /** Flat ground with a trunk and a collision-solid leaf canopy beside the descent corridor. */
    private static final class WoodedTerrain extends World {
        private final double[][] trees = {{3, 0, 3, 4, 5, 4}, {2, 4, 2, 5, 7, 5}};

        @Override public boolean chunkLoaded(int x, int z) { return true; }

        @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) {
            if (minY < 0) return true;
            for (double[] box : trees) {
                if (maxX > box[0] && minX < box[3] && maxY > box[1] && minY < box[4]
                        && maxZ > box[2] && minZ < box[5]) return true;
            }
            return false;
        }

        @Override public OptionalDouble findHighestCollisionTopExcluding(double minX, double minY,
                double minZ, double maxX, double maxY, double maxZ, Set<Vector3i> excluded) {
            double top = minY <= 0 && maxY >= 0 ? 0 : Double.NEGATIVE_INFINITY;
            for (double[] box : trees) {
                if (maxX > box[0] && minX < box[3] && maxZ > box[2] && minZ < box[5]
                        && minY <= box[4] && maxY >= box[4]) top = Math.max(top, box[4]);
            }
            return Double.isFinite(top) ? OptionalDouble.of(top) : OptionalDouble.empty();
        }
    }

    private static PhysicsState state(double height, double verticalSpeed, double horizontalSpeed, double fallDistance) {
        return new PhysicsState(new Vector3d(.5, height, .5),
                new Vector3d(0, verticalSpeed, horizontalSpeed), false, fallDistance);
    }

    private static PhysicsState next(PhysicsResult result) {
        return new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance());
    }

    @Test void steepDescentHasProvenZeroDamageCandidateAmongTrees() {
        var world = new WoodedTerrain();
        assertTrue(world.isBoxColliding(3.1, 1, 3.1, 3.9, 2, 3.9), "fixture contains a trunk");
        assertTrue(world.isBoxColliding(2.1, 5, 2.1, 2.9, 6, 2.9), "fixture contains leaves");
        var initial = state(16, -.8, .5, 50);
        assertEquals(0, controller.emergencyDamage(world, initial, 0, -30, false));
        var decision = controller.forceLand(world, initial, 0, 10, false, 0, 16);
        assertEquals(ElytraFlightController.Phase.FORCED_LANDING, decision.phase());
        assertFalse(decision.firework());
        assertEquals(0, controller.emergencyDamage(world, initial, decision.yaw(), decision.pitch(), false));
    }

    @Test void collisionFreeWindowDoesNotHideUnrecoverableFallDamage() {
        var world = new WoodedTerrain();
        var physics = new GlidingPhysics(0, 90, false);
        var high = state(200, -2, .1, 50);
        var simulated = high;
        for (int tick = 0; tick < 24; tick++) simulated = next(physics.step(world, simulated, .299, .6));
        // Put the same trajectory just above ground at the horizon, with no time left to pull up.
        var initial = state(200 - simulated.position().y + .1, -2, .1, 50);
        simulated = initial;
        for (int tick = 0; tick < 24; tick++) {
            var result = physics.step(world, simulated, .299, .6);
            assertFalse(result.onGround() || result.collidedX() || result.collidedZ(), "tick " + tick);
            simulated = next(result);
        }
        assertEquals(.1, simulated.position().y, 1e-8);
        assertTrue(simulated.velocity().y < -.5);
        for (float pitch : PITCHES) {
            assertTrue(new GlidingPhysics(0, pitch, false).step(world, simulated, .299, .6).onGround(),
                    "every recovery pitch must hit ground before resetting fall distance");
        }
        double damage = controller.emergencyDamage(world, initial, 0, 90, false);
        assertTrue(Double.isFinite(damage), "Infinity means unproven, not known damage");
        assertTrue(damage > 0);
    }

    @Test void forceLandChoosesLeastDamageWhenEveryCandidateIsHarmful() {
        var world = new WoodedTerrain();
        var initial = new PhysicsState(new Vector3d(3.5, 4, .5), new Vector3d(0, -2, 3), false, 20);
        var controller = new ElytraFlightController(MovementParameters.DEFAULT, new DamageParameters(4, 3, 0), 20);
        var target = ElytraFlightController.selectForcedLandingTarget(world, initial, 0, false);
        float heading = (float) Math.toDegrees(Math.atan2(initial.position().x - target.x,
                target.z - initial.position().z));
        if (Math.hypot(initial.position().x - target.x, initial.position().z - target.z) < .01) heading = 0;
        double minimum = Double.POSITIVE_INFINITY;
        double maximum = 0;
        for (float offset : YAW_OFFSETS) {
            float yaw = heading + offset;
            while (yaw >= 180) yaw -= 360;
            while (yaw < -180) yaw += 360;
            for (float pitch : PITCHES) {
                double damage = controller.emergencyDamage(world, initial, yaw, pitch, false);
                assertTrue(Double.isFinite(damage) && damage > 0, "candidate " + yaw + ", " + pitch);
                minimum = Math.min(minimum, damage);
                maximum = Math.max(maximum, damage);
            }
        }
        assertTrue(maximum > minimum, "fixture must distinguish lower and higher damage actions");
        var decision = controller.forceLand(world, initial, 0, 10, false, 0, 16);
        assertEquals(ElytraFlightController.Phase.FORCED_LANDING, decision.phase());
        assertFalse(decision.firework());
        assertEquals(minimum, controller.emergencyDamage(world, initial, decision.yaw(), decision.pitch(), false));
    }
}
