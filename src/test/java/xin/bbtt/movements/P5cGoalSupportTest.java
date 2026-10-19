package xin.bbtt.movements;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.KnownLandingReserve;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Synthetic fourth-field goal: centre loaded does not imply its landing footprint is known. */
final class P5cGoalSupportTest {
    private static final Vector3d GOAL = new Vector3d(175.5, 60, -31.5);
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0, Map.of("level", "0"), "empty", 0, false, "LAVA");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");

    private static KnownLandingReserve.Result reserve(int surplus) {
        return new KnownLandingReserve.Result(KnownLandingReserve.Status.VALID,
                new Vector3d(4.590, 35, -1.513), 1, 1, surplus, true, true);
    }

    private static Object returnSite(ElytraFlyMovement movement) throws Exception {
        var field = ElytraFlyMovement.class.getDeclaredField("reserveReturnSite");
        field.setAccessible(true);
        return field.get(movement);
    }

    @Test void route180With64RocketsAndUnloadedGoalFootprintContinuesNotReturns() throws Exception {
        new MovementSync();
        var movement = new ElytraFlyMovement(GOAL);
        // Actual field departure is only ~7.49 blocks from the retained support.
        var departure = new Vector3d(-2.616, 37.04, -1.606);
        assertTrue(departure.distance(reserve(61).returnSite()) < 8);
        for (boolean centreLoaded : new boolean[] {false, true}) {
            World world = new World() {
                @Override public boolean chunkLoaded(int x, int z) { return centreLoaded && x <= 10; }
                @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 31 ? LAVA : AIR; }
            };
            assertTrue(reserve(61).canContinue());
            assertFalse(movement.rejectUnsupportedGoal(world, GOAL, reserve(61)));
            assertNull(returnSite(movement), "unknown footprint must not turn positive-surplus departure into return");
            assertFalse(movement.isFinished());
            assertEquals(GOAL, movement.getTarget());
        }
    }

    @Test void confirmedLavaGoalExplicitlyFailsInsteadOfReturningOrClaimingArrival() throws Exception {
        new MovementSync();
        var movement = new ElytraFlyMovement(GOAL);
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 31 ? LAVA : AIR; }
        };
        assertTrue(movement.rejectUnsupportedGoal(world, GOAL, reserve(61)), "confirmed unsupported goal must be rejected");
        assertNull(returnSite(movement), "goal geometry is not a resource return trigger");
        assertFalse(movement.completedPlannedReturn());
        var install = ElytraFlyMovement.class.getDeclaredMethod("installResourceReturn", KnownLandingReserve.Result.class);
        install.setAccessible(true);
        assertTrue((boolean) install.invoke(movement, reserve(0)), "real resource threshold still starts return");
        assertFalse(movement.rejectUnsupportedGoal(world, GOAL, reserve(0)));
        assertEquals(reserve(0).returnSite(), returnSite(movement));
    }

    @Test void rejectedAirborneGoalRemainsActiveUntilResourceReturn() throws Exception {
        var sync = new MovementSync();
        sync.onGround.set(false);
        var movement = new ElytraFlyMovement(GOAL);
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 31 ? LAVA : AIR; }
        };
        for (int tick = 0; tick < 20; tick++) {
            assertTrue(movement.rejectUnsupportedGoal(world, GOAL, reserve(61)));
            assertFalse(movement.isFinished(), "rejection must retain airborne control ownership");
            assertNull(returnSite(movement), "positive surplus must not start return");
        }
        var rejected = ElytraFlyMovement.class.getDeclaredField("unsupportedGoalRejected");
        rejected.setAccessible(true);
        assertTrue(rejected.getBoolean(movement), "retain explicit failed goal outcome");
        var install = ElytraFlyMovement.class.getDeclaredMethod("installResourceReturn", KnownLandingReserve.Result.class);
        install.setAccessible(true);
        assertTrue((boolean) install.invoke(movement, reserve(0)));
        assertEquals(reserve(0).returnSite(), returnSite(movement));
        assertFalse(movement.isFinished(), "return must finish physically, not at installation");
    }

    @Test void supportedGoalDoesNotFailOrStartReturn() throws Exception {
        new MovementSync();
        var movement = new ElytraFlyMovement(GOAL);
        var stone = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 60 ? stone : AIR; }
        };
        assertFalse(movement.rejectUnsupportedGoal(world, GOAL, reserve(61)));
        assertNull(returnSite(movement));
        assertFalse(movement.isFinished());
    }
}
