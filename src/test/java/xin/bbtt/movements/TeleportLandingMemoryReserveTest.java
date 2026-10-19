package xin.bbtt.movements;

import java.util.Arrays;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.flight.DangerousEntryGate;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Synthetic A-to-B teleport probe, not a reconstruction of the field's chunks. */
final class TeleportLandingMemoryReserveTest {
    private static final BlockState STONE = new BlockState("minecraft:netherrack", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0, Map.of("level", "0"), "empty", 0, false, "LAVA");

    @Test void rememberedDistantShoreDoesNotExcludeNewTeleportDeparture() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y >= 120 || p.x < 0 && p.y < 36
                        || p.x >= 224 && p.x < 240 && p.y < 34) return STONE;
                return p.y < 31 ? LAVA : AIR;
            }
        };
        var memory = new LandingMemory();
        var a = new Vector3d(227.5, 34, 21.5);
        memory.takeoff(world, a);
        memory.landed(world, a);
        memory.observe(world, a);
        assertFalse(memory.snapshot().isEmpty());
        var b = new PhysicsState(new Vector3d(-3.5, 36, -1.5), new Vector3d(), true);
        var launch = LowAltitudeFlightPlan.deployment(world, b, MovementParameters.DEFAULT, -90, 0);
        assertNotNull(launch);
        int[] flights = new int[64];
        Arrays.fill(flights, 1);
        for (LandingMemory remembered : new LandingMemory[] {new LandingMemory(), memory}) {
            var resources = new HazardRecovery.Context(flights, 0, false, remembered, 0);
            var sites = DangerousEntryGate.knownSites(world, b, resources);
            assertTrue(sites.contains(b.position()), "current B support must remain eligible with A memory");
            assertTrue(KnownLandingReserve.launchReserve(world, launch, MovementParameters.DEFAULT,
                    resources, -90, 0, sites), "A memory must not reject a supported high-resource B launch");
            var lowResources = new HazardRecovery.Context(new int[] {1, 1, 1}, 0, false, remembered, 0);
            assertFalse(KnownLandingReserve.launchReserve(world, launch, MovementParameters.DEFAULT,
                    lowResources, -90, 0, sites), "neither fresh nor stale memory grants free launch reserve");
        }
    }
}
