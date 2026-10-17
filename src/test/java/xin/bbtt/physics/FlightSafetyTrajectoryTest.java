package xin.bbtt.physics;

import java.util.concurrent.atomic.AtomicInteger;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.physics.FlightSafety.Impact;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class FlightSafetyTrajectoryTest {
    private final World world = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
    };
    private final FlightSafety safety = new FlightSafety(MovementParameters.DEFAULT, DamageParameters.NONE, 20);

    private static PhysicsState initial(double verticalSpeed) {
        return new PhysicsState(new Vector3d(8, 100, 8), new Vector3d(0, verticalSpeed, 0), false, 12);
    }

    private static PhysicsResult advance(PhysicsState state, boolean wall, boolean ground) {
        return new PhysicsResult(state.position().add(state.velocity()), state.velocity(), wall, false, ground,
                state.fallDistance() + 1);
    }

    @Test void genericPredictionHandsFinalHorizonStateToRecovery() {
        AtomicInteger ticks = new AtomicInteger();
        AtomicInteger recoveries = new AtomicInteger();
        Impact expected = new Impact(false, 7, 3);
        Impact actual = safety.predictEmergencyImpact(world, initial(-.5), 12, 0, false,
                (w, state, width, height) -> {
                    assertEquals(.299, width);
                    assertEquals(.6, height);
                    ticks.incrementAndGet();
                    return advance(state, false, false);
                }, (w, state, yaw, attached) -> {
                    recoveries.incrementAndGet();
                    assertSame(world, w);
                    assertEquals(88, state.position().y);
                    assertEquals(36, state.fallDistance());
                    assertEquals(12, yaw);
                    assertFalse(attached);
                    return expected;
                });
        assertEquals(expected, actual);
        assertEquals(24, ticks.get());
        assertEquals(1, recoveries.get());
    }

    @Test void unknownStepStopsBeforeCollisionDamageAndRecovery() {
        AtomicInteger ticks = new AtomicInteger();
        World unknown = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d position) {
                fail("Unknown terrain must not be classified for damage");
                return null;
            }
        };
        assertEquals(Impact.UNKNOWN, safety.predictEmergencyImpact(unknown, initial(-1), 0, 0, false,
                (w, state, width, height) -> {
                    ticks.incrementAndGet();
                    return advance(state, true, true);
                }, (w, state, yaw, attached) -> fail("Unexpected recovery")));
        assertEquals(1, ticks.get());
    }

    @Test void rawDamageStopsEvenWhenFullyMitigated() {
        FlightSafety immune = new FlightSafety(MovementParameters.DEFAULT, new DamageParameters(0, 0, 4), 20);
        PhysicsState initial = new PhysicsState(new Vector3d(8, 100, 8), new Vector3d(2, -1, 0), false);
        AtomicInteger ticks = new AtomicInteger();
        Impact impact = immune.predictEmergencyImpact(world, initial, 0, 0, false,
                (w, state, width, height) -> {
                    ticks.incrementAndGet();
                    return new PhysicsResult(state.position(), new Vector3d(0, -1, 0), true, false, false);
                }, (w, state, yaw, attached) -> fail("Unexpected recovery"));
        assertEquals(17, impact.raw());
        assertEquals(0, impact.reduced());
        assertEquals(1, ticks.get());
    }

    @Test void harmlessBrushContinuesUntilRecoveryAndKeepsTrajectory() {
        AtomicInteger ticks = new AtomicInteger();
        FlightSafety.CheckedTrajectory checked = safety.simulate(world, initial(-1),
                (w, state, width, height) -> {
                    int tick = ticks.incrementAndGet();
                    assertEquals(11 + tick, state.fallDistance());
                    return new PhysicsResult(state.position(), new Vector3d(0, tick == 3 ? -.49 : -.5, 0),
                            tick == 1, false, false, state.fallDistance() + 1);
                }, 60, 0, 0, false, true);
        assertEquals(Impact.SAFE, checked.terminalImpact());
        assertEquals(3, checked.trajectory().size());
        assertEquals(0, checked.trajectory().firstCollision());
        assertEquals(15, checked.finalState().fallDistance());
    }

    @Test void lavaWithoutCollisionStopsLegacyPredictionBeforeRecovery() {
        World lava = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d position) {
                return new xin.bbtt.Block.BlockState("lava", 0, java.util.Map.of(), "empty", 0, false, "");
            }
        };
        AtomicInteger ticks = new AtomicInteger();
        Impact impact = safety.predictEmergencyImpact(lava, initial(-1), 0, 0, false,
                (w, state, width, height) -> {
                    ticks.incrementAndGet();
                    return advance(state, false, false);
                }, (w, state, yaw, attached) -> fail("Hazard cannot be recovered as safe"));
        assertTrue(impact.unknown(), "Forbidden lava cells must never win emergency ranking");
        assertEquals(1, ticks.get());
    }

    @Test void exhaustionIsNotRecoveryButEmergencyHorizonCanBeSafe() {
        FlightSafety.CheckedTrajectory checked = safety.simulate(world, initial(-.5),
                (w, state, width, height) -> advance(state, false, false), 60, 0, 0, false, true);
        assertNull(checked.terminalImpact());
        assertEquals(60, checked.trajectory().size());
        assertEquals(Impact.SAFE, safety.predictEmergencyImpact(world, initial(-.49), 0, 0, false,
                (w, state, width, height) -> advance(state, false, false),
                (w, state, yaw, attached) -> fail("Unexpected recovery")));
    }
}
