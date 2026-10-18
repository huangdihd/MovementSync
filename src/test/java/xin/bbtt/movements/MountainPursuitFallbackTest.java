package xin.bbtt.movements;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.flight.FlightContext;
import xin.bbtt.flight.FlightContextBuilder;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class MountainPursuitFallbackTest {
    private static final class Mountain extends World {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d p) {
            return p.y < 80
                    ? new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE")
                    : new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
        }
    }

    @Test void reportedSlowDescentHasPoweredLocalExitWithoutSpendingRecoveryBudget() {
        var world = new Mountain();
        var state = new PhysicsState(new Vector3d(-243, 81.72, 20.45),
                new Vector3d(-.306, -.1757, -.06942), false, 0, PlayerPose.GLIDING);
        var budget = new HazardRecovery.Budget(HazardRecovery.DECISION_STEP_BUDGET);
        var reserve = HazardRecovery.assess(world, state, MovementParameters.DEFAULT,
                new HazardRecovery.Context(new int[] {1, 1}, 0, false, null), budget);
        assertEquals(HazardRecovery.Status.NOT_EXPOSED, reserve.status());
        assertEquals(0, budget.steps(), "solid mountain support cannot starve powered recovery candidates");
        assertNull(reserve.decision(), "NOT_EXPOSED is not a steering fallback");
        var context = new FlightContext(world, state, MovementParameters.DEFAULT, DamageParameters.NONE, 20,
                new Vector3d(-275, 80, 28.5), 90, 0, 2, false, 2, false, Map.of());
        var config = new FlightContextBuilder().addStandard().build();
        var decision = ElytraFlyMovement.localPursuitDecision(context, context.target(), config);
        assertNotNull(decision);
        assertTrue(decision.firework());
        assertNotNull(ElytraFlyMovement.airbornePursuitPreview(context, decision, config));
        assertTrue(ElytraFlyMovement.lowDecisionSafe(context, decision, config));
        var unknown = new FlightContext(new World(), state, MovementParameters.DEFAULT, DamageParameters.NONE, 20,
                context.target(), 90, 0, 2, false, 2, false, Map.of());
        assertNull(ElytraFlyMovement.airbornePursuitPreview(unknown, decision, config),
                "previous controls must be revalidated against actual loaded terrain");
    }
}
