package xin.bbtt.movements;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.pathfinding.StandablePositionResolver;
import xin.bbtt.physics.ElytraFlightController;
import xin.bbtt.physics.KnownLandingReserve;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Synthetic ledge, not a reconstruction of the unavailable field chunk snapshot. */
final class P5dExactGoalStandabilityTest {
    private static final Vector3d GOAL = new Vector3d(175.5, 60, -31.5);
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0, Map.of("level", "0"), "empty", 0, false, "LAVA");

    private static World terrain(String kind, boolean loaded) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return loaded; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                int x = (int) Math.floor(p.x), y = (int) Math.floor(p.y), z = (int) Math.floor(p.z);
                if (y < 42) return STONE;
                if (kind.equals("air")) return AIR;
                if (kind.equals("lava")) return y == 59 || y == 60 ? LAVA : AIR;
                if (x == 175 && z == -32 && y < 60) return STONE;
                if (kind.equals("solid") && y == 60) return STONE;
                // A taller adjacent step is outside the exact walking body, but blocks a broad landing area.
                if (kind.equals("step") && x == 176 && z == -32 && y < 62) return STONE;
                return AIR;
            }
        };
    }

    private static boolean rejected(World world, Vector3d goal) {
        new MovementSync();
        var movement = new ElytraFlyMovement(goal);
        var reserve = new KnownLandingReserve.Result(KnownLandingReserve.Status.VALID,
                new Vector3d(4.590, 35, -1.513), 1, 1, 61, true, true);
        boolean rejected = movement.rejectUnsupportedGoal(world, goal, reserve);
        assertFalse(movement.isFinished());
        assertFalse(movement.completedPlannedReturn());
        assertEquals(goal, movement.getTarget());
        return rejected;
    }

    @Test void narrowY60LedgeAndAdjacentStepAboveY42TerrainAreValidWalkingGoals() {
        for (String kind : new String[]{"ledge", "step"}) {
            World world = terrain(kind, true);
            assertEquals(60, StandablePositionResolver.feetY(world, 175, 60, -32).orElseThrow());
            assertNull(ElytraFlightController.landingSite(world, GOAL.x, GOAL.z, GOAL.y));
            assertFalse(rejected(world, GOAL), kind);
        }
    }

    @Test void lowerTerrainMustNotSubstituteForExactGoalHeight() {
        World world = terrain("air", true);
        assertEquals(42, ElytraFlightController.landingSite(world, GOAL.x, GOAL.z, GOAL.y).y);
        assertTrue(rejected(world, GOAL));
    }

    @Test void loadedLavaSolidAndUnsupportedAirRejectButUnknownDoesNot() {
        for (String kind : new String[]{"lava", "solid", "air"}) {
            assertTrue(rejected(terrain(kind, true), GOAL), kind);
            assertFalse(rejected(terrain(kind, false), GOAL), kind);
        }
    }

    @Test void exactNonCenteredBodyCannotBeRoundedOntoLedge() {
        assertTrue(rejected(terrain("ledge", true), new Vector3d(176.4, 60, -31.5)));
        assertFalse(rejected(terrain("ledge", true), new Vector3d(175.9, 60, -31.5)));
    }
}
