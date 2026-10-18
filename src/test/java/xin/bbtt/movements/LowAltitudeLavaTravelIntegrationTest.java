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
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.events.ReceivePacketEvent;
import xin.bbtt.physics.GlidingPhysics;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Real travel orchestration and controls; only the server's acknowledgements are simulated. */
final class LowAltitudeLavaTravelIntegrationTest extends ElytraCommandTestSupport {
    private static final double HALF_WIDTH = (double) 0.6f / 2;
    private static final int ROCKET_ID = 9001;
    private static final BlockState STONE = new BlockState(
            "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
            "minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final BlockState LAVA = new BlockState(
            "minecraft:lava", 0, Map.of(), "empty", 0, false, "LAVA");
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

    /** A wide lava sea separates the launch shore from a ten-block wall/target plateau. */
    private static final class LavaSea extends World {
        @Override public boolean chunkLoaded(int x, int z) { return x >= -2 && x <= 12 && z >= -2 && z <= 2; }
        @Override public java.util.Set<ChunkPosition> loadedChunks() {
            var chunks = new java.util.HashSet<ChunkPosition>();
            for (int x = -2; x <= 12; x++) for (int z = -2; z <= 2; z++)
                chunks.add(new ChunkPosition(x, z));
            return java.util.Set.copyOf(chunks);
        }
        @Override public BlockState getBlockStateAt(Vector3d p) {
            if (p.y >= 60 || p.y < 29 || p.x >= 0 && p.x < 16 && Math.abs(p.z) < 8 && p.y < 32
                    || p.x >= 128 && p.y < 42) return STONE;
            return p.y < 32 ? LAVA : AIR;
        }
    }

    @Test
    void sufficientRocketsCrossRoofedLavaSeaAndLandOnOppositeShore() throws Exception {
        exerciseTravel(64, true);
    }

    @Test
    void noRocketsStayOnOrReturnToSupportedLaunchShore() throws Exception {
        exerciseTravel(0, false);
    }

    private void exerciseTravel(int rockets, boolean mustArrive) throws Exception {
        LavaSea world = new LavaSea();
        Vector3d start = new Vector3d(8.5, 32, .5);
        Vector3i goal = new Vector3i(144, 42, 0);
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.position.set(new Vector3d(start));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        inventory[36] = rockets == 0 ? null : item("firework_rocket", rockets);
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

        boolean jumped = false;

        int physicsTicks = 0;
        int tick = 0;
        try {
            for (; tick < 400 && !travel.isFinished(); tick++) {
                // Wait for genuine worker results, never replace/publish routes ourselves.
                // This keeps the simulated 20 Hz clock independent of machine throughput.
                awaitPlanning(travel);
                Object child = field(travel, "child");
                if (child instanceof ElytraFlyMovement) awaitPlanning(child);
                travel.onTick();
                var input = movementSync.snapshotInput();
                var packet = movementSync.pollBeforeMovementPacket();
                while (packet != null) {
                    Bot.INSTANCE.getSession().send(packet);
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
                        rocketRequests++;
                        inventory[36] = rockets == rocketRequests ? null : item("firework_rocket", rockets - rocketRequests);
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
                if (!mustArrive) assertTrue(p.x <= 16 - HALF_WIDTH,
                        "empty inventory must not enter the lava sea: " + diagnostic);
                assertTrue(p.y >= 32 && p.y + height <= 60 + 1.e-6, "floor/roof penetration: " + diagnostic);
                assertTrue(world.chunkLoaded((int) Math.floor(p.x) >> 4, (int) Math.floor(p.z) >> 4),
                        "entered unknown terrain: " + diagnostic);
                if (!gliding && p.y > start.y + .5) jumped = true;
                physicsTicks++;
                if (gliding) flightTicks++;
                // Evaluate the swept body against lava, not just the endpoint's feet.
                var context = new xin.bbtt.flight.FlightContext(world, state,
                        movementSync.getMovementParameters(), xin.bbtt.physics.DamageParameters.NONE,
                        20, new Vector3d(goal.x + .5, goal.y, goal.z + .5),
                        movementSync.yaw.get(), movementSync.pitch.get(),
                        movementSync.countHotbarFireworks(), movementSync.hasAttachedFirework(),
                        2, true, Map.of());
                assertEquals(0, xin.bbtt.physics.PublicFlightBridge.hazard(context, state, next, true).raw(), diagnostic);
                assertEquals(0, xin.bbtt.physics.PublicFlightBridge.hazard(context, state, next, false).raw(), diagnostic);
                // Every position after setup comes exclusively from the shared physics step.
                movementSync.position.set(p);
                movementSync.velocity.set(next.velocity());
                movementSync.onGround.set(next.onGround());
                movementSync.fallDistance.set(next.fallDistance());
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
            assertTrue(physicsTicks > 0, "must execute real physics: " + diagnostic);
            Vector3d endpoint = new Vector3d(goal.x + .5, goal.y, goal.z + .5);
            boolean arrived = movementSync.position.get().distance(endpoint) <= 1.5;
            assertTrue(movementSync.onGround.get(), "must finish on support: " + diagnostic);
            assertFalse(movementSync.isGliding(), diagnostic);
            assertTrue(world.isOnGround(movementSync.position.get()), diagnostic);
            if (mustArrive) {
                assertTrue(arrived, "sufficient resources must cross, not reject: " + diagnostic);
                assertTrue(travel.isFinished(), diagnostic);
                assertTrue(deployments > 0 && flightTicks > 0 && rocketRequests > 0, diagnostic);
                assertTrue(jumped && groundedAcknowledgements > 0, diagnostic);
            } else {
                assertFalse(arrived, diagnostic);
                assertEquals(0, rocketRequests, diagnostic);
                assertNotNull(travel.lowFlightFailure(), "no silent timeout or false success: " + diagnostic);
                assertTrue(movementSync.position.get().x <= 16 - HALF_WIDTH,
                        "rejection must remain on the launch shore: " + diagnostic);
            }
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
        if (pending instanceof CompletableFuture<?> future) future.get(120, TimeUnit.SECONDS);
    }
}
