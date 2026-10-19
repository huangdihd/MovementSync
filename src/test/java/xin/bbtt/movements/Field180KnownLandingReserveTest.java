package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import org.cloudburstmc.nbt.NbtMap;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.listeners.RegistryDataListener;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Synthetic field geometry, not a reconstruction of unavailable server chunks. */
final class Field180KnownLandingReserveTest {
    private static final BlockState STONE = new BlockState("minecraft:netherrack", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0, Map.of("level", "0"), "empty", 0, false, "LAVA");

    @Test void seaLaunchStillRejectsThreeRocketsAndAdmitsSixtyFour() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y >= 120 || (p.x < 0 || p.x >= 500) && p.y < 40) return STONE;
                return p.y < 31 ? LAVA : AIR;
            }
        };
        var ground = new PhysicsState(new Vector3d(-8.5, 40, .5), new Vector3d(), true);
        var launch = LowAltitudeFlightPlan.deployment(world, ground, MovementParameters.DEFAULT, -90, 0);
        assertNotNull(launch);
        for (int count : new int[] {3, 64}) {
            int[] flights = new int[count];
            java.util.Arrays.fill(flights, 1);
            assertEquals(count == 64, KnownLandingReserve.launchReserve(world, launch,
                    MovementParameters.DEFAULT, new HazardRecovery.Context(flights, 0, false, null, 0),
                    -90, 0, List.of(ground.position())), "sea launch inventory=" + count);
        }
    }

    @Test @SuppressWarnings("unchecked")
    void lavaNearNetherAscentRetainsHighInventoryLaunchReserve() throws Exception {
        Field types = RegistryDataListener.class.getDeclaredField("dimensionTypes");
        types.setAccessible(true);
        var dimensions = (List<NbtMap>) types.get(null);
        var saved = new ArrayList<>(dimensions);
        Field index = RegistryDataListener.class.getDeclaredField("currentDimensionIndex");
        index.setAccessible(true);
        int oldIndex = index.getInt(null);
        try {
            dimensions.clear();
            dimensions.add(NbtMap.builder().putBoolean("has_ceiling", true).putInt("min_y", 0)
                    .putInt("height", 256).putInt("logical_height", 128).build());
            index.setInt(null, 0);
            World world = new World() {
                @Override public boolean chunkLoaded(int x, int z) { return true; }
                @Override public BlockState getBlockStateAt(Vector3d p) {
                    if (p.y >= 120) return STONE;
                    if (p.z > 8) return p.y < 31 ? LAVA : AIR;
                    double floor = 36 + Math.min(24, Math.max(0, Math.floor(p.x / 7)));
                    return p.y < floor ? STONE : AIR;
                }
            };
            var state = new PhysicsState(new Vector3d(-3.5, 36, -1.5), new Vector3d(), true);
            var goal = new Vector3d(175, 60, -32);
            int[] flights = new int[64];
            java.util.Arrays.fill(flights, 1);
            var resources = new HazardRecovery.Context(flights, 0, false, null, 0);
            assertFalse(TravelMovement.isOpen(world, state.position()));
            assertTrue(KnownLandingReserve.nearLava(world, state), "ordinary synthetic terrain did not exercise this gate");
            var future = LowAltitudeFlightPlan.submit(world, state, goal, MovementParameters.DEFAULT, false, resources);
            try {
                var plan = future.get(110, TimeUnit.SECONDS);
                assertNotNull(plan, future.detail());
                assertTrue(LowAltitudeFlightPlan.usableLaunch(world, state, goal,
                        MovementParameters.DEFAULT, resources, plan, future.status()),
                        "64 rockets and a safe climbing corridor must not be rejected by a level-only excursion");
                var launch = LowAltitudeFlightPlan.deployment(world, state, MovementParameters.DEFAULT, plan.yaw(), plan.pitch());
                assertTrue(KnownLandingReserve.launchReserve(world, launch, MovementParameters.DEFAULT,
                        resources, plan.yaw(), plan.pitch(), List.of(state.position())));
            } finally { future.cancel(true); }
        } finally {
            dimensions.clear();
            dimensions.addAll(saved);
            index.setInt(null, oldIndex);
        }
    }
}
