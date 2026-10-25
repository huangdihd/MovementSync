package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.OptionalInt;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;
import org.cloudburstmc.nbt.NbtMap;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.EntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.MetadataTypes;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.type.ByteEntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerState;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundSetEntityDataPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Entity.Entity;
import xin.bbtt.Entity.EntityRegistry;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.listeners.EntityPacketListener;
import xin.bbtt.listeners.RegistryDataListener;
import xin.bbtt.mcbot.events.ReceivePacketEvent;
import xin.bbtt.physics.GlidingPhysics;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Real travel orchestration and controls; only the server's acknowledgements are simulated. */
final class LocalGotoSyntheticTerrainEndToEndTest extends ElytraCommandTestSupport {
    private static final double HALF_WIDTH = (double) 0.6f / 2;
    private static final int ROCKET_ID = 9001;
    private static final BlockState STONE = new BlockState(
            "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
            "minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
    private List<NbtMap> dimensions;
    private List<NbtMap> savedDimensions;
    private Field dimensionIndex;
    private int savedIndex;

    @BeforeEach @SuppressWarnings("unchecked")
    void installCeilingDimension() throws Exception {
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

    @AfterEach
    void restoreDimension() throws Exception {
        dimensions.clear();
        dimensions.addAll(savedDimensions);
        dimensionIndex.setInt(null, savedIndex);
    }

    /** All planning and collision queries see the same synthetic voxels. */
    private static final class SyntheticTerrain extends World {
        @Override public boolean chunkLoaded(int x, int z) { return true; }

        static double floorAt(double x) {
            // Climb 24 blocks after the passage, reaching the Y=60 landing shelf at X=168.
            return 36 + Math.max(0, Math.min(24, Math.floor((x - 120) / 2)));
        }

        @Override public BlockState getBlockStateAt(Vector3d p) {
            // The full-height wall cannot be overflown. Its offset eight-block opening
            // leads into an eight-block-wide passage, then a stepped climb to the shelf.
            boolean wall = p.x >= 60 && p.x < 64 && (p.z < -40 || p.z >= -32);
            boolean narrow = p.x >= 64 && p.x < 120 && (p.z < -40 || p.z >= -32);
            return p.y < floorAt(p.x) || p.y >= 76 || p.z < -48 || p.z >= 16 || wall || narrow
                    ? STONE : AIR;
        }
    }

    @Test
    void localGotoFliesFromMinus3Point5Y36To175Y60ThroughWallAndNarrowPassage() throws Exception {
        SyntheticTerrain world = new SyntheticTerrain();
        Vector3d start = new Vector3d(-3.5, 36, -1.5);
        Vector3i goal = new Vector3i(175, 60, -32);
        assertEquals(24, goal.y - start.y);
        assertSame(STONE, world.getBlockStateAt(new Vector3d(62, 42, -1.5)));
        assertSame(AIR, world.getBlockStateAt(new Vector3d(62, 42, -36)));
        assertSame(STONE, world.getBlockStateAt(new Vector3d(100, 42, -41)));
        assertSame(STONE, world.getBlockStateAt(new Vector3d(175.5, 59, -31.5)));
        assertSame(AIR, world.getBlockStateAt(new Vector3d(175.5, 60, -31.5)));
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.position.set(new Vector3d(start));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        assertFalse(TravelMovement.isOpen(world, start));
        assertTrue(world.isOnGround(start));
        TravelMovement travel = new TravelMovement(goal, movementSync.beginStaticNavigationRequest(goal), true);
        travel.init();
        PlayerPose pose = PlayerPose.STANDING;
        int deployments = 0;
        int rocketRequests = 0;
        int rocketTicks = 0;
        int flightTicks = 0;
        int groundedAcknowledgements = 0;
        int lastRocketTick = -40;
        boolean sawLocalPrefix = false;
        boolean jumped = false;
        boolean traversedPassageInFlight = false;
        double flightDistance = 0;
        int tick = 0;
        try {
            for (; tick < 1600 && !travel.isFinished(); tick++) {
                // Wait for genuine worker results, never replace/publish routes ourselves.
                // This keeps the simulated 20 Hz clock independent of machine throughput.
                awaitPlanning(travel);
                Object child = field(travel, "child");
                if (child instanceof ElytraFlyMovement) awaitPlanning(child);
                travel.onTick();
                child = field(travel, "child");
                if (child instanceof ElytraFlyMovement) {
                    Object controller = field(child, "localGoto");
                    var await = controller.getClass().getDeclaredMethod("awaitIdle", long.class, TimeUnit.class);
                    await.setAccessible(true);
                    await.invoke(controller, 30L, TimeUnit.SECONDS);
                    var result = ((xin.bbtt.flight.LocalGotoController) controller).latest();
                    if (result != null && result.status() == xin.bbtt.flight.LocalGotoController.Status.SAFE_PREFIX)
                        sawLocalPrefix = true;
                }
                assertNull(travel.lowFlightFailure(), "flight rejected at tick=" + tick);
                var input = movementSync.snapshotInput();
                var packet = movementSync.pollBeforeMovementPacket();
                while (packet != null) {
                    assertEquals(MovementSync.BeforeMovementDispatch.SENT,
                            movementSync.sendBeforeMovementPacket(packet, packet));
                    if (packet instanceof ServerboundPlayerCommandPacket command
                            && command.getState() == PlayerState.START_ELYTRA_FLYING) {
                        assertFalse(movementSync.onGround.get(), "deployment requires the real jump");
                        assertTrue(movementSync.velocity.get().y <= 0, "deploy on descent");
                        deployments++;
                        sharedFlags(true);
                    } else if (packet instanceof ServerboundUseItemPacket) {
                        assertTrue(movementSync.isGliding(), "rocket request must follow deployment");
                        assertEquals(0, rocketTicks, "do not stack attached rockets");
                        assertTrue(movementSync.countHotbarFireworks() > 0);
                        assertTrue(tick - lastRocketTick >= 40, "rocket cooldown bypass at tick=" + tick);
                        lastRocketTick = tick;
                        rocketRequests++;
                        inventory[36] = item("firework_rocket", 64 - rocketRequests);
                        movementSync.inventoryManager.setContainerItems(0, inventory);
                        // Flight-1 lifetime: 20 + random(6) + random(7); choose a deterministic 30.
                        rocketTicks = 30;
                        Entity rocket = new Entity(ROCKET_ID, new UUID(0, ROCKET_ID), EntityType.FIREWORK_ROCKET,
                                movementSync.position.get().x, movementSync.position.get().y,
                                movementSync.position.get().z, 0, 0, 0, new Vector3d());
                        rocket.getMetadata().put("attached_to_target", OptionalInt.of(movementSync.entityId));
                        world.getEntities().put(ROCKET_ID, rocket);
                    }
                    packet = movementSync.pollBeforeMovementPacket();
                }
                boolean gliding = movementSync.isGliding();
                PhysicsState state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                        movementSync.onGround.get(), movementSync.fallDistance.get(), pose);
                double height = (double) (float) pose.height();
                PhysicsResult next = gliding
                        ? new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(),
                                movementSync.hasAttachedFirework(), input, movementSync.getMovementParameters())
                                .step(world, state, HALF_WIDTH, height)
                        : new WalkingPhysics(input, movementSync.yaw.get(), true, movementSync.getMovementParameters())
                                .step(world, state, HALF_WIDTH, height);
                String diagnostic = "tick=" + tick + " position=" + next.position() + " goal=" + goal;
                assertFalse(next.collidedX() || next.collidedZ(), "horizontal impact: " + diagnostic);
                Vector3d p = next.position();
                assertTrue(p.isFinite(), diagnostic);
                assertTrue(p.y >= SyntheticTerrain.floorAt(p.x + HALF_WIDTH)
                                && p.y + height <= 76 + 1.e-6, "floor/roof penetration: " + diagnostic);
                assertTrue(world.chunkLoaded((int) Math.floor(p.x) >> 4, (int) Math.floor(p.z) >> 4),
                        "entered unknown terrain: " + diagnostic);
                if (!gliding && p.y > start.y + .5) jumped = true;
                if (gliding) {
                    flightTicks++;
                    flightDistance += Math.hypot(p.x - state.position().x, p.z - state.position().z);
                    if (p.x >= 64 && p.x < 120) {
                        assertTrue(p.z >= -40 + HALF_WIDTH && p.z <= -32 - HALF_WIDTH,
                                "must traverse the narrow opening, not clip its walls: " + diagnostic);
                        traversedPassageInFlight = true;
                    }
                }
                // Every position after setup comes exclusively from the shared physics step.
                movementSync.position.set(p);
                movementSync.velocity.set(next.velocity());
                movementSync.onGround.set(next.onGround());
                movementSync.fallDistance.set(next.fallDistance());
                movementSync.completePhysicsStep();
                pose = pose.afterTravel(world, p, gliding, false, input.sneak());
                if (rocketTicks > 0 && --rocketTicks == 0) world.getEntities().remove(ROCKET_ID);
                if (gliding && next.onGround()) {
                    assertTrue(movementSync.isGliding(), "contact alone does not clear client flight");
                    sharedFlags(false);
                    groundedAcknowledgements++;
                    world.getEntities().remove(ROCKET_ID);
                    rocketTicks = 0;
                }
            }
            String diagnostic = "ticks=" + tick + " position=" + movementSync.position.get()
                    + " velocity=" + movementSync.velocity.get() + " deployments=" + deployments
                    + " rockets=" + rocketRequests + " flightTicks=" + flightTicks;
            assertTrue(travel.isFinished(), "travel did not complete: " + diagnostic);
            assertTrue(jumped, "ground launch must actually jump: " + diagnostic);
            assertTrue(deployments > 0, "walking fallback is not a flight E2E: " + diagnostic);
            assertTrue(rocketRequests > 0, "must exercise real rocket requests: " + diagnostic);
            assertTrue(packets.stream().anyMatch(ServerboundUseItemPacket.class::isInstance));
            assertTrue(flightDistance > start.distance(new Vector3d(goal.x + .5, goal.y, goal.z + .5)) * .7,
                    "must fly the route rather than immediately land and walk: " + diagnostic);
            assertTrue(groundedAcknowledgements > 0, "must physically land and receive shared_flags");
            assertTrue(movementSync.onGround.get(), diagnostic);
            assertFalse(movementSync.isGliding(), diagnostic);
            assertEquals(goal.y, movementSync.position.get().y, 1.e-6, diagnostic);
            assertTrue(movementSync.position.get().distance(new Vector3d(goal.x + .5, goal.y, goal.z + .5)) <= 1.5,
                    "completion must be at the physical endpoint: " + diagnostic);
            assertTrue(sawLocalPrefix, "must exercise the local worker, not legacy control");
            assertTrue(traversedPassageInFlight, "must fly through the wall and narrow passage");
        } finally {
            travel.onStop();
            world.getEntities().remove(ROCKET_ID);
        }
    }

    private void sharedFlags(boolean gliding) {
        int flagsId = EntityRegistry.Instance.getEntity("PLAYER").getMetadataKeys().indexOf("shared_flags");
        assertTrue(flagsId >= 0);
        var packet = new ClientboundSetEntityDataPacket(movementSync.entityId, new EntityMetadata<?, ?>[]{
                new ByteEntityMetadata(flagsId, MetadataTypes.BYTE, (byte) (gliding ? 0x80 : 0))
        });
        new EntityPacketListener().OnSetEntityMetadata(
                new ReceivePacketEvent<>(packet, ClientboundSetEntityDataPacket.class));
        assertEquals(gliding, movementSync.isGliding());
    }

    private static Object field(Object owner, String name) throws Exception {
        Field field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(owner);
    }

    private static void awaitPlanning(Object owner) throws Exception {
        Object pending = field(owner, "lowPlan");
        if (pending instanceof CompletableFuture<?> future) future.get(30, TimeUnit.SECONDS);
    }
}
