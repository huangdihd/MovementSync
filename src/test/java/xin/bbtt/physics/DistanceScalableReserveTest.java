package xin.bbtt.physics;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

class DistanceScalableReserveTest {
    private static World sea() {
        var stone = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        var lava = new BlockState("minecraft:lava", 0, Map.of(), "empty", 0, false, "LAVA");
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y >= 120 || (p.x < 0 || p.x >= 600) && p.y < 34) return stone;
                return p.y < 31 ? lava : super.getBlockStateAt(p);
            }
        };
    }
    private static HazardRecovery.Context resources(int count, int cooldown) {
        int[] flights = new int[count];
        Arrays.fill(flights, 1);
        return new HazardRecovery.Context(flights, cooldown, false, null, 0);
    }
    private static PhysicsState state(double x) {
        return new PhysicsState(new Vector3d(x, 37, .5), new Vector3d(1.35, .04, 0),
                false, 1, PlayerPose.GLIDING);
    }
    @Test void sixHundredWideSeaAtOneNinetyContinuesWith64ButRejects3AsResources() {
        for (int count : new int[]{64, 3}) {
            var result = KnownLandingReserve.assess(sea(), state(190), MovementParameters.DEFAULT,
                    resources(count, 1), List.of(new Vector3d(-8.5, 34, .5)), 10, 1);
            assertEquals(KnownLandingReserve.Status.VALID, result.status(), result.toString());
            assertEquals(count == 64, result.canContinue(), result.toString());
            assertTrue(result.requiredRockets() > 3);
        }
    }
    @Test void loggedReversingStateAt187BlocksRetainsPositiveReserve() {
        var stone = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return p.x < 240 && p.y < 34 ? stone : super.getBlockStateAt(p);
            }
        };
        var state = new PhysicsState(new Vector3d(417.2, 36.87, 10.92),
                new Vector3d(1.349, .0391, -.05856), false, 1, PlayerPose.GLIDING);
        var result = KnownLandingReserve.assess(world, state, MovementParameters.DEFAULT,
                resources(64, 1), List.of(new Vector3d(229.8, 34, 20.76)), 10, 1);
        assertTrue(result.canContinue(), result.toString());
    }
    @Test void sixHundredDistanceIncludesReversalClimbAndCooldown() {
        var result = KnownLandingReserve.assess(sea(), state(591.5), MovementParameters.DEFAULT,
                resources(64, 20), List.of(new Vector3d(-8.5, 34, .5)), 10, 1);
        assertTrue(result.canContinue(), result.toString());
        assertTrue(result.requiredRockets() > 30);
    }
    @Test void estimateChargesClimbReversalCooldownAndActualComponents() {
        int[] flights = new int[64];
        Arrays.fill(flights, 1);
        int base = FireworkSchedule.reserveRockets(flights, 0, 0, 10, 190, 0, 0);
        assertTrue(FireworkSchedule.reserveRockets(flights, 40, 0, 10, 190, 12, 1.35) > base);
        Arrays.fill(flights, 3);
        assertTrue(FireworkSchedule.reserveRockets(flights, 0, 0, 10, 190, 0, 0) < base);
        assertEquals(-1, FireworkSchedule.reserveRockets(flights, 0, 0, 21, 190, 0, 0));
    }
    @Test void unknownRouteIsEstimationFailureNotInventoryShortage() {
        var result = KnownLandingReserve.assess(new World(), state(190), MovementParameters.DEFAULT,
                resources(64, 0), List.of(new Vector3d(-8.5, 34, .5)), 10, 1);
        assertFalse(result.canContinue());
        assertNotEquals(KnownLandingReserve.Status.VALID, result.status());
        assertEquals(-1, result.requiredRockets());
    }
    @Test void badFarAnchorCannotDragDownNearestSupportedAnchor() {
        var near = new Vector3d(-8.5, 34, .5);
        var far = new Vector3d(900, 34, .5);
        var alone = KnownLandingReserve.assess(sea(), state(20), MovementParameters.DEFAULT,
                resources(64, 0), List.of(near), 10, 1);
        var mixed = KnownLandingReserve.assess(sea(), state(20), MovementParameters.DEFAULT,
                resources(64, 0), List.of(far, near), 10, 1);
        assertTrue(alone.canContinue(), alone.toString());
        assertEquals(alone, mixed);
        assertEquals(near, mixed.returnSite());
    }
}
