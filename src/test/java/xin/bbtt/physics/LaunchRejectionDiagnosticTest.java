package xin.bbtt.physics;

import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Synthetic diagnostics only: unavailable actual180 field state is not reconstructed. */
final class LaunchRejectionDiagnosticTest {
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final PhysicsState STATE = new PhysicsState(new Vector3d(0, 80, 0), new Vector3d(), false);

    private World air() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return AIR; }
        };
    }

    @Test void invalidResourcesAreReportedBeforeScheduling() {
        for (var resources : List.of(new HazardRecovery.Context(new int[] {256}, 0, false, null, 0),
                new HazardRecovery.Context(new int[] {1}, 7, false, null, -1))) {
            var result = KnownLandingReserve.launchAssessment(air(), STATE, MovementParameters.DEFAULT,
                    resources, 0, 0, List.of());
            assertEquals(KnownLandingReserve.LaunchReason.INVALID_RESOURCES, result.reason());
            assertTrue(result.attemptedPitches().isEmpty());
            assertFalse(KnownLandingReserve.launchReserve(air(), STATE, MovementParameters.DEFAULT,
                    resources, 0, 0, List.of()));
        }
    }

    @Test void blockedExcursionRecordsAllClimbAttemptsWithoutClaimingShortage() {
        var wall = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return STONE; }
        };
        var result = KnownLandingReserve.launchAssessment(wall, STATE, MovementParameters.DEFAULT,
                HazardRecovery.Context.EMPTY, 0, 0, List.of());
        assertEquals(KnownLandingReserve.LaunchReason.BLOCKED_EXCURSION, result.reason());
        assertEquals(List.of(0f, -10f, -30f), result.attemptedPitches());
        assertNull(result.reserve());
        assertTrue(result.excursionTicks() > 0);
        assertFalse(result.canContinue());
    }

    @Test void terrainSafeExcursionWithNoAnchorStopsWithoutTryingClimbs() {
        var resources = new HazardRecovery.Context(new int[] {1, 1, 1}, 0, false, null, 0);
        var result = KnownLandingReserve.launchAssessment(air(), STATE, MovementParameters.DEFAULT,
                resources, 0, 0, List.of());
        assertEquals(KnownLandingReserve.LaunchReason.NO_ANCHOR, result.reason());
        assertEquals(List.of(0f), result.attemptedPitches());
        assertEquals(40, result.excursionTicks());
        assertEquals(1, result.consumedRockets());
        assertEquals(0, result.cooldownTicks());
        assertEquals(0, result.boostTicks());
        assertEquals(KnownLandingReserve.Status.NO_SITE, result.reserve().status());
    }

    @Test void onlyProvenEstimateWithInsufficientSurplusIsInventoryMargin() {
        for (var status : List.of(KnownLandingReserve.Status.UNPROVEN,
                KnownLandingReserve.Status.BLOCKED_OR_UNKNOWN)) {
            var result = new KnownLandingReserve.Result(status, new Vector3d(), -1, 1,
                    Integer.MIN_VALUE, true, false);
            assertEquals(KnownLandingReserve.LaunchReason.UNPROVEN_RETURN,
                    KnownLandingReserve.launchReason(result));
        }
        var margin = new KnownLandingReserve.Result(KnownLandingReserve.Status.VALID,
                new Vector3d(), 2, 1, 0, true, true);
        assertEquals(KnownLandingReserve.LaunchReason.INVENTORY_MARGIN, KnownLandingReserve.launchReason(margin));
        var admitted = new KnownLandingReserve.Result(KnownLandingReserve.Status.VALID,
                new Vector3d(), 2, 1, 1, true, true);
        assertEquals(KnownLandingReserve.LaunchReason.ADMITTED, KnownLandingReserve.launchReason(admitted));
    }
}
