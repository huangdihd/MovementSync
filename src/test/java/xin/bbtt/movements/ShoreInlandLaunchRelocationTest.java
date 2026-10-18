package xin.bbtt.movements;

import org.cloudburstmc.nbt.NbtMap;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerState;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.listeners.RegistryDataListener;
import xin.bbtt.physics.DeploymentSafety;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.tasks.updateMotionTask;
import xin.bbtt.world.World;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.*;

/** Real travel, walking and deployment ticks. No injected plans, poses or launch positions. */
class ShoreInlandLaunchRelocationTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final BlockState LAVA = new BlockState("lava", 0, Map.of("level", "0"), "empty", 0, false, "LAVA");
    private static final Vector3d START = new Vector3d(245.5, 33, 20.5);
    private List<NbtMap> dimensions;
    private List<NbtMap> savedDimensions;
    private Field dimensionIndex;
    private int savedIndex;

    @BeforeEach @SuppressWarnings("unchecked")
    void ceilingDimension() throws Exception {
        Field field = RegistryDataListener.class.getDeclaredField("dimensionTypes");
        field.setAccessible(true);
        dimensions = (List<NbtMap>) field.get(null);
        savedDimensions = new ArrayList<>(dimensions);
        dimensionIndex = RegistryDataListener.class.getDeclaredField("currentDimensionIndex");
        dimensionIndex.setAccessible(true);
        savedIndex = dimensionIndex.getInt(null);
        dimensions.clear();
        dimensions.add(NbtMap.builder().putBoolean("has_ceiling", true)
                .putInt("min_y", 0).putInt("height", 256).putInt("logical_height", 128).build());
        dimensionIndex.setInt(null, 0);
    }

    @AfterEach void restoreDimension() throws Exception {
        dimensions.clear();
        dimensions.addAll(savedDimensions);
        dimensionIndex.setInt(null, savedIndex);
    }

    private World install(boolean narrow, int landingX, int rockets) throws Exception {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                boolean shore = narrow ? p.x >= 245 && p.x < 246 && p.z >= 20 && p.z < 21 : p.x < 246;
                if (p.y >= 120 || shore && p.y < 33 || p.x >= landingX && p.y < 40) return STONE;
                return p.y < 31 ? LAVA : AIR;
            }
        };
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
        movementSync.position.set(new Vector3d(START));
        movementSync.velocity.set(new Vector3d(0, -.0784, 0));
        movementSync.onGround.set(true);
        movementSync.yaw.set(-95.4657f);
        movementSync.pitch.set(0f);
        if (rockets > 0) inventory[36] = item("firework_rocket", rockets);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        assertFalse(TravelMovement.isOpen(world, START));
        assertFalse(DeploymentSafety.assess(world, state(), movementSync.getMovementParameters(), -95.4657f, 0).safe(),
                "the recorded shore itself must fail deployment safety");
        return world;
    }

    private PhysicsState state() {
        return new PhysicsState(movementSync.position.get(), movementSync.velocity.get(), movementSync.onGround.get(),
                movementSync.fallDistance.get());
    }

    @Test void shorterViableCrossingWalksInlandBeforeActualDeployment() throws Exception {
        World world = install(false, 350, 64);
        Vector3i goal = new Vector3i(365, 40, 9);
        TravelMovement travel = new TravelMovement(goal, movementSync.beginStaticNavigationRequest(goal), true);
        var physics = new updateMotionTask();
        Method tick = updateMotionTask.class.getDeclaredMethod("tick");
        tick.setAccessible(true);
        boolean walkedInland = false;
        boolean jumped = false;
        boolean deployed = false;
        travel.init();
        try {
            for (int i = 0; i < 400 && !travel.isFinished() && !deployed; i++) {
                awaitWorkers(travel);
                Vector3d before = new Vector3d(movementSync.position.get());
                travel.onTick();
                if (!jumped && movementSync.onGround.get() && before.x < START.x - .5) walkedInland = true;
                int packetStart = packets.size();
                tick.invoke(physics);
                Vector3d after = movementSync.position.get();
                assertTrue(after.distance(before) < 1, "approach must move by physics, not teleport");
                assertTrue(after.y >= 33 && after.x < 246, "approach/deployment must stay on the safe shore");
                assertTrue(movementSync.fallDistance.get() < 3, "no damaging fall during relocation");
                if (!movementSync.onGround.get() && !movementSync.isGliding()) {
                    assertTrue(walkedInland, "no jump until physically walking inland");
                    jumped = true;
                }
                for (Object packet : packets.subList(packetStart, packets.size())) {
                    if (packet instanceof ServerboundPlayerCommandPacket command
                            && command.getState() == PlayerState.START_ELYTRA_FLYING) {
                        assertTrue(walkedInland && jumped, "deployment must follow the real approach and jump");
                        assertTrue(DeploymentSafety.assess(world, state(), movementSync.getMovementParameters(),
                                movementSync.yaw.get(), movementSync.pitch.get()).safe(), "actual launch must be safe");
                        // Explicit simulated server acknowledgement; local prediction is not acceptance.
                        movementSync.applySharedFlags((byte) 0x80);
                        assertTrue(movementSync.isGliding());
                        deployed = true;
                    }
                }
            }
            assertTrue(walkedInland, "Travel must choose and physically reach an inland launch");
            assertTrue(deployed, "viable shorter crossing must actually send START_ELYTRA_FLYING");
        } finally { travel.onStop(); }
    }

    @Test void recordedLongGoalWithoutResourcesStillRejectsInsteadOfClaimingSixHundredBlockFlight() throws Exception {
        install(false, 760, 0);
        rejectWithoutJump(new Vector3i(778, 40, -31));
    }

    @Test void recordedLongCrossingDoesNotBypassFullResourceProofByRelocating() throws Exception {
        // The reported goal is about 535 horizontal blocks from the recorded shore,
        // often called the 600-block case. Do not replace its full proof with a partial route.
        install(false, 760, 64);
        rejectWithoutJump(new Vector3i(778, 40, -31));
    }

    @Test void allNearbyLaunchesDangerousRejectWithoutJumpOrDeploymentRetries() throws Exception {
        install(true, 350, 64);
        rejectWithoutJump(new Vector3i(365, 40, 9));
    }

    private void rejectWithoutJump(Vector3i goal) throws Exception {
        TravelMovement travel = new TravelMovement(goal, movementSync.beginStaticNavigationRequest(goal), true);
        Method tick = updateMotionTask.class.getDeclaredMethod("tick");
        tick.setAccessible(true);
        var physics = new updateMotionTask();
        travel.init();
        try {
            // Continue ticking after rejection to catch accidental retry loops.
            for (int i = 0; i < 120; i++) {
                awaitWorkers(travel);
                travel.onTick();
                tick.invoke(physics);
                assertTrue(movementSync.onGround.get(), "rejected shore must never jump");
                assertEquals(33, movementSync.position.get().y, 1e-6);
                assertFalse(movementSync.isGliding());
            }
            assertNotNull(travel.lowFlightFailure(), "failure must remain explicit, not a fabricated flight proof");
            assertTrue(packets.stream().noneMatch(packet -> packet instanceof ServerboundPlayerCommandPacket command
                    && command.getState() == PlayerState.START_ELYTRA_FLYING), "no deployment or retries");
        } finally { travel.onStop(); }
    }

    private static void awaitWorkers(Object owner) throws Exception {
        for (Field field : owner.getClass().getDeclaredFields()) {
            field.setAccessible(true);
            Object value = field.get(owner);
            if (value instanceof CompletableFuture<?> future) future.get(150, TimeUnit.SECONDS);
            if (field.getName().equals("child") && value instanceof ElytraFlyMovement) awaitWorkers(value);
        }
    }
}
