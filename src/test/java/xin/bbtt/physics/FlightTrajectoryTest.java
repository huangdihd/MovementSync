package xin.bbtt.physics;

import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class FlightTrajectoryTest {
    @Test void includesStopTickAndNeverStepsAgain() {
        var initial = new PhysicsState(new Vector3d(1, 50, 2), new Vector3d(0, -.1, 1), false, 4);
        var calls = new AtomicInteger();
        var stops = new AtomicInteger();
        PhysicsStepper physics = (world, state, width, height) -> {
            int tick = calls.getAndIncrement();
            assertTrue(tick < 3, "physics must not run after the stop tick");
            assertEquals(.299, width);
            assertEquals(.6, height);
            assertEquals(initial.position().add(0, -tick, tick), state.position());
            assertEquals(4 + tick, state.fallDistance());
            assertFalse(state.onGround());
            return new PhysicsResult(state.position().add(0, -1, 1), state.velocity(),
                    tick == 1, tick == 2, false, 5 + tick);
        };
        var trajectory = FlightTrajectory.simulate(new World(), initial, physics, 24, (state, result) -> {
            stops.incrementAndGet();
            return result.collidedZ();
        });
        assertSame(initial, trajectory.initial());
        assertEquals(3, calls.get());
        assertEquals(3, stops.get());
        assertEquals(3, trajectory.size());
        assertEquals(1, trajectory.firstCollision());
        assertTrue(trajectory.result(2).collidedZ());
    }

    @Test void recordsEachPhysicalCollisionKindIncludingTheStoppingTick() {
        for (int kind = 0; kind < 3; kind++) {
            final int collision = kind;
            var calls = new AtomicInteger();
            var trajectory = FlightTrajectory.simulate(new World(),
                    new PhysicsState(new Vector3d(), new Vector3d(), false), (world, state, w, h) -> {
                        assertEquals(0, calls.getAndIncrement());
                        return new PhysicsResult(state.position(), state.velocity(),
                                collision == 0, collision == 1, collision == 2);
                    }, 24, (state, result) -> true);
            assertEquals(1, trajectory.size());
            assertEquals(0, trajectory.firstCollision());
            assertEquals(1, calls.get());
        }
    }

    @Test void collisionFreeSimulationUsesExactlyTheHorizonIncludingZero() {
        for (int horizon : new int[]{0, 1, 24}) {
            var calls = new AtomicInteger();
            var trajectory = FlightTrajectory.simulate(new World(),
                    new PhysicsState(new Vector3d(), new Vector3d(), false), (world, state, w, h) -> {
                        calls.incrementAndGet();
                        return new PhysicsResult(state.position(), state.velocity(), false, false);
                    }, horizon, (state, result) -> false);
            assertEquals(horizon, calls.get());
            assertEquals(horizon, trajectory.size());
            assertEquals(-1, trajectory.firstCollision());
        }
    }
}
