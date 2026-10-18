package xin.bbtt.physics;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class HazardRecoveryTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0, Map.of(), "empty", 0, false, "");

    private World sea(double roof, boolean wall) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y >= roof || wall && p.x >= 12 && p.x < 13) return STONE;
                if (p.y < 32) return LAVA;
                return super.getBlockStateAt(p);
            }
        };
    }

    private PhysicsState flying(double y) {
        return new PhysicsState(new Vector3d(8, y, 8), new Vector3d(.8, 0, 0), false, 0, PlayerPose.GLIDING);
    }

    @Test void lavaAt31Feet32CannotBeAdmittedBySixTickClearance() {
        World world = sea(48, true);
        assertFalse(HazardRecovery.assess(world, flying(32), MovementParameters.DEFAULT).admissible());
        assertNull(LowAltitudePlanner.plan(world, flying(32), new Vector3d(10, 32, 8), MovementParameters.DEFAULT));
    }

    @Test void reportedSeaHeightAndInjectedZeroVelocityAreExcludedBeforeDeployment() {
        World world = sea(48, true);
        PhysicsState cruising = flying(32.31);
        PhysicsState stopped = new PhysicsState(cruising.position(), new Vector3d(), false, 0, PlayerPose.GLIDING);
        assertFalse(HazardRecovery.assess(world, cruising, MovementParameters.DEFAULT).admissible());
        assertFalse(HazardRecovery.assess(world, stopped, MovementParameters.DEFAULT).admissible());
        assertNull(LowAltitudePlanner.plan(world, cruising, new Vector3d(20, 40, 8), MovementParameters.DEFAULT));
    }

    @Test void broadSeaWithWallAndRoofNeedsTerminalRecoveryNotFinitePrefix() {
        World world = sea(48, true);
        assertEquals(HazardRecovery.Status.UNPROVEN,
                HazardRecovery.assess(world, flying(40), MovementParameters.DEFAULT).status());
        assertNull(LowAltitudePlanner.plan(world, flying(40), new Vector3d(20, 40, 8), MovementParameters.DEFAULT));
    }

    @Test void lowRoofMakesUnpoweredLavaRecoveryImpossible() {
        World world = sea(33, false);
        assertFalse(HazardRecovery.assess(world, flying(32.2), MovementParameters.DEFAULT).admissible());
        assertNull(LowAltitudePlanner.plan(world, flying(32.2), new Vector3d(10, 32.2, 8), MovementParameters.DEFAULT));
    }

    @Test void loadedDrySupportDoesNotDemandPermanentAirborneFlight() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return p.y < 32 ? STONE : super.getBlockStateAt(p);
            }
        };
        assertEquals(HazardRecovery.Status.NOT_EXPOSED,
                HazardRecovery.assess(world, flying(34), MovementParameters.DEFAULT).status());
    }

    @Test void nearbyDryBankCanProvideTerminalUnpoweredRecovery() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y < 32) return p.x >= 9 ? STONE : LAVA;
                return super.getBlockStateAt(p);
            }
        };
        assertEquals(HazardRecovery.Status.RECOVERABLE,
                HazardRecovery.assess(world, flying(40), MovementParameters.DEFAULT).status());
    }

    @Test void unknownTerrainCannotProveRecovery() {
        assertFalse(HazardRecovery.assess(new World(), flying(40), MovementParameters.DEFAULT).admissible());
    }
    private World coast(double distance, boolean blocked) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y >= 48 || blocked && p.x >= 60 && p.x < 61) return STONE;
                if (p.y < 32) return p.x >= distance ? STONE : LAVA;
                return super.getBlockStateAt(p);
            }
        };
    }

    @Test void distantCoastUsesFiniteRocketsAndProducesExecutableRecovery() {
        World world = coast(180, false);
        int[] rockets = new int[12];
        java.util.Arrays.fill(rockets, 1);
        var context = new HazardRecovery.Context(rockets, 0, false, null);
        var result = HazardRecovery.assess(world, flying(40), MovementParameters.DEFAULT, context);
        assertEquals(HazardRecovery.Status.RECOVERABLE, result.status());
        assertNotNull(result.decision());
        assertTrue(result.target().x >= 180);
        assertNotNull(LowAltitudePlanner.plan(world, flying(40), new Vector3d(180, 32, 8),
                MovementParameters.DEFAULT, context));
        assertFalse(HazardRecovery.assess(coast(600, false), flying(40), MovementParameters.DEFAULT,
                new HazardRecovery.Context(new int[] {1}, 0, false, null)).admissible());
        assertFalse(HazardRecovery.assess(coast(180, true), flying(40), MovementParameters.DEFAULT, context).admissible());
    }

    @Test void cooldownAndCandidateConsumptionCannotInventRocketReserve() {
        var context = new HazardRecovery.Context(new int[] {1, 2}, 0, false, null);
        var remaining = context.afterTicks(6, true);
        assertArrayEquals(new int[] {2}, remaining.rocketFlightDurations());
        assertEquals(34, remaining.cooldownTicks());
        assertTrue(remaining.attached());
        assertEquals(15, remaining.guaranteedBoostTicks(), "spent rocket retains its minimum-life continuation");
        assertThrows(IllegalArgumentException.class, () -> remaining.afterTicks(1, true));
        int[] copy = context.rocketFlightDurations();
        copy[0] = 99;
        assertEquals(1, context.rocketFlightDurations()[0]);
        assertFalse(HazardRecovery.assess(coast(180, false), flying(40), MovementParameters.DEFAULT,
                new HazardRecovery.Context(new int[] {1, 1, 1, 1}, 200, false, null)).admissible());
    }

}
