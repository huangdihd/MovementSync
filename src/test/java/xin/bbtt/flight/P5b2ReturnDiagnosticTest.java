package xin.bbtt.flight;

import java.util.Arrays;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class P5b2ReturnDiagnosticTest {
    private World shore() {
        var stone = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        var lava = new BlockState("minecraft:lava", 0, Map.of(), "empty", 0, false, "LAVA");
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.x < 240 && p.y < 34) return stone;
                if (p.y < 31) return lava;
                return super.getBlockStateAt(p);
            }
        };
    }

    @Test void failedTargetPlaneEstimateWithSixtyRocketsUsesTouchdownBeforeReturning() {
        int[] flights = new int[60];
        Arrays.fill(flights, 1);
        var resources = new HazardRecovery.Context(flights, 0, false, null);
        var state = new PhysicsState(new Vector3d(405.4, 37.87, 11.29),
                new Vector3d(1.596, -.004242, -.09322), false, 1.00424, PlayerPose.GLIDING);
        var site = new Vector3d(229.8, 34, 20.76);
        var estimate = new KnownLandingReserve.Result(KnownLandingReserve.Status.UNPROVEN,
                site, -1, 1, Integer.MIN_VALUE, true, true);
        var result = ReserveMonitor.resolveReturnEstimate(shore(), state, MovementParameters.DEFAULT,
                resources, estimate);
        assertTrue(result.canContinue(), result.toString());
        assertEquals(60 - result.requiredRockets() - 1, result.surplusRockets());
        assertEquals("POSITIVE_SURPLUS", ReserveMonitor.returnTrigger(result));
        var plan = KnownReturnPlan.preview(shore(), state, MovementParameters.DEFAULT, resources, site, 10);
        assertTrue(plan.proven());
        assertEquals(plan.steps().size(), plan.trajectoryTicks());
        assertTrue(plan.trajectoryTicks() <= KnownReturnPlan.MAX_TICKS);
        assertTrue(plan.simulatedSteps() >= plan.trajectoryTicks());
        assertEquals(plan.requiredRockets(), result.requiredRockets());
    }

    @Test void failedFallbackStillForbidsExposureAndDiagnosticsDistinguishFuelFromProof() {
        var site = new Vector3d(229.8, 34, 20.76);
        var estimate = new KnownLandingReserve.Result(KnownLandingReserve.Status.UNPROVEN,
                site, -1, 1, Integer.MIN_VALUE, true, true);
        var state = new PhysicsState(new Vector3d(405, 38, 11), new Vector3d(1.6, 0, 0), false);
        assertSame(estimate, ReserveMonitor.resolveReturnEstimate(new World(), state,
                MovementParameters.DEFAULT, HazardRecovery.Context.EMPTY, estimate));
        assertTrue(estimate.shouldReturn());
        assertEquals("RESERVE_UNPROVEN", ReserveMonitor.returnTrigger(estimate));
        var threshold = new KnownLandingReserve.Result(KnownLandingReserve.Status.VALID,
                site, 2, 1, 0, true, true);
        assertSame(threshold, ReserveMonitor.resolveReturnEstimate(shore(), state,
                MovementParameters.DEFAULT, HazardRecovery.Context.EMPTY, threshold));
        assertEquals("RESOURCE_THRESHOLD", ReserveMonitor.returnTrigger(threshold));
    }
}
