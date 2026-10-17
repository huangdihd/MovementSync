package xin.bbtt.physics;

import java.util.OptionalDouble;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class VanillaGlideModeTrajectoryTest {
    private static final class Air extends World {
        boolean loaded = true;
        @Override public boolean chunkLoaded(int x, int z) { return loaded; }
        @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
            return false;
        }
        @Override public OptionalDouble findHighestCollisionTop(double x, double y, double z,
                double X, double Y, double Z) { return OptionalDouble.empty(); }
    }

    @Test void predictionRetainsFirstUnsafeTickAndCostRejectsIt() {
        var mode = new VanillaGlideMode(MovementParameters.DEFAULT);
        for (String reason : new String[]{"unknown", "envelope", "below", "above"}) {
            var world = new Air();
            world.loaded = !reason.equals("unknown");
            double y = reason.equals("below") ? World.getMinWorldY() - 2
                    : reason.equals("above") ? World.getMaxWorldY() + 2 : 50;
            var initial = new PhysicsState(new Vector3d(8, y, 8), new Vector3d(0, -.1, .2), false);
            var target = new Vector3d(8, 60, 100);
            boolean emergency = reason.equals("envelope");
            var trajectory = mode.predict(world, initial, 0, 0, false, 24, target, emergency);
            assertEquals(1, trajectory.size(), reason);
            assertEquals(-1, trajectory.firstCollision(), reason + " is not a physical collision");
            var expected = new GlidingPhysics(0, 0, false).step(world, initial, .299, .6);
            assertEquals(expected, trajectory.result(0), reason + " must retain the unsafe result");
            assertEquals(Double.POSITIVE_INFINITY, BuiltinFlightCost.evaluate(world, trajectory,
                    target, 2, 50, 0, 0, false, false, false, 24, emergency), reason);
        }
    }

    @Test void safePredictionRunsTheFullHorizonAndScoresFinitely() {
        var world = new Air();
        var initial = new PhysicsState(new Vector3d(8, 100, 8), new Vector3d(0, -.1, .2), false);
        var target = new Vector3d(8, 100, 100);
        var trajectory = new VanillaGlideMode(MovementParameters.DEFAULT)
                .predict(world, initial, 0, 0, false, 24, target, false);
        assertEquals(24, trajectory.size());
        assertEquals(-1, trajectory.firstCollision());
        assertTrue(Double.isFinite(BuiltinFlightCost.evaluate(world, trajectory,
                target, 2, 100, 0, 0, false, false, false, 24, false)));
    }
}
