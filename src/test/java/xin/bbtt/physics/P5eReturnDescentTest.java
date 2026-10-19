package xin.bbtt.physics;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Rounded tick1295 of the absent-far-shore physical regression, before descent is spent. */
class P5eReturnDescentTest {
    @Test void approachFlaresBeforeAccumulatedFallCrossesSafeLimit() {
        var stone = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        var lava = new BlockState("minecraft:lava", 0, Map.of(), "empty", 0, false, "LAVA");
        var world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y >= 120 || p.x < 0 && p.y < 40) return stone;
                return p.y < 31 ? lava : super.getBlockStateAt(p);
            }
        };
        var initial = new PhysicsState(new Vector3d(-2.005, 58.23, .5),
                new Vector3d(-1.545, -.5322, 0), false, 2.0334, PlayerPose.GLIDING);
        var plan = KnownReturnPlan.preview(world, initial, MovementParameters.DEFAULT,
                HazardRecovery.Context.EMPTY, new Vector3d(-8.5, 40, .5), 0);
        assertTrue(plan.proven(), plan.reason());
        assertEquals(0, plan.requiredRockets(), "approach must not restart the rocket schedule");
        assertTrue(plan.steps().get(0).control().pitch() < 0, "flare before the unsafe descent tick");
        var state = initial;
        for (var step : plan.steps()) {
            assertEquals(state, step.before());
            var next = new GlidingPhysics(step.control().yaw(), step.control().pitch(),
                    step.resource().powered(), PlayerInput.NONE, MovementParameters.DEFAULT)
                    .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            assertTrue(next.fallDistance() <= MovementParameters.DEFAULT.safeFallDistance(),
                    "unsafe airborne descent: " + next);
            assertEquals(0, FlightHazards.damage(world, state, next, DamageParameters.NONE, true).raw());
            assertEquals(0, FlightHazards.damage(world, state, next, DamageParameters.NONE, false).raw());
            state = FlightSafety.nextState(next);
            assertEquals(step.after(), state, "stored witness must reproduce actual physics");
        }
        assertTrue(state.onGround());
        assertTrue(state.position().x < 0);
        assertEquals(40, state.position().y);
        var replay = KnownReturnPlan.replay(world, initial, MovementParameters.DEFAULT, plan);
        assertTrue(replay.proven(), replay.reason());
    }
}
