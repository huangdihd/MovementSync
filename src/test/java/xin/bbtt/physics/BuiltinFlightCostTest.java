package xin.bbtt.physics;

import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class BuiltinFlightCostTest {
    private static final class Terrain extends World {
        boolean loaded = true;
        boolean sweptCollision;
        int boxes;
        int supports;
        @Override public boolean chunkLoaded(int x, int z) { return loaded; }
        @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) {
            boxes++;
            return sweptCollision;
        }
        @Override public OptionalDouble findHighestCollisionTop(double x, double y, double z,
                double X, double Y, double Z) {
            supports++;
            return OptionalDouble.empty();
        }
    }

    @Test void rejectionShortCircuitsBeforeReadingAnotherTickOrRunningPhysicsAgain() {
        for (String reason : new String[]{"unknown", "wallX", "wallZ", "swept", "ground", "envelope", "below", "above"}) {
            var world = new Terrain();
            world.loaded = !reason.equals("unknown");
            world.sweptCollision = reason.equals("swept");
            double y = reason.equals("below") ? World.getMinWorldY() - 1
                    : reason.equals("above") ? World.getMaxWorldY() : 50;
            var initial = new PhysicsState(new Vector3d(8, y, 8), new Vector3d(0, -.1, .2), false);
            var calls = new AtomicInteger();
            var trajectory = FlightTrajectory.simulate(world, initial, (w, state, width, height) -> {
                assertEquals(0, calls.getAndIncrement(), reason);
                return new PhysicsResult(state.position(), state.velocity(), reason.equals("wallX"),
                        reason.equals("wallZ"), reason.equals("ground"));
            }, 24, (state, result) -> true);
            // Deliberately request more ticks than stored: rejection must return on tick zero.
            for (int scoring = 0; scoring < 2; scoring++) {
                world.boxes = 0;
                world.supports = 0;
                assertEquals(Double.POSITIVE_INFINITY, BuiltinFlightCost.evaluate(world, trajectory,
                        new Vector3d(8, 60, 100), 2, 50, 0, 0, false, false, false, 24,
                        reason.equals("envelope")), reason);
                assertEquals(reason.equals("unknown") || reason.startsWith("wall") ? 0 : 1,
                        world.boxes, reason + " must skip later collision queries");
                assertEquals(reason.equals("envelope") ? 1 : 0, world.supports, reason);
                assertEquals(1, calls.get(), reason + " must score cached physics");
            }
        }
    }

    @Test void finiteScoringReusesTheEntireTrajectory() {
        var world = new Terrain();
        var calls = new AtomicInteger();
        var initial = new PhysicsState(new Vector3d(8, 50, 8), new Vector3d(0, 0, 1), false);
        var trajectory = FlightTrajectory.simulate(world, initial, (w, state, width, height) -> {
            calls.incrementAndGet();
            return new PhysicsResult(state.position().add(0, 0, 1), state.velocity(), false, false);
        }, 3, (state, result) -> false);
        for (int scoring = 0; scoring < 2; scoring++) {
            assertEquals(27, BuiltinFlightCost.evaluate(world, trajectory, new Vector3d(8, 50, 20),
                    2, 50, 0, 0, false, false, false, 3, false));
            assertEquals(3, calls.get());
        }
    }
}
