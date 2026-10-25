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
final class KnownReturnLavaSeaPhysicalRegressionTest extends ElytraCommandTestSupport {
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

    private static final BlockState LAVA = new BlockState(
            "minecraft:lava", 0, Map.of("level", "0"), "empty", 0, false, "LAVA");

    /** Knowledge grows only from actual position; the far shore starts outside loaded chunks. */
    private final class LavaSea extends World {
        private final boolean farShore;
        private int loadedThrough = 15;
        LavaSea(boolean farShore) { this.farShore = farShore; }
        void observe() {
            loadedThrough = Math.max(loadedThrough,
                    ((int) Math.floor(movementSync.position.get().x) >> 4) + 16);
        }
        @Override public boolean chunkLoaded(int x, int z) { return x <= loadedThrough; }
        @Override public BlockState getBlockStateAt(Vector3d p) {
            if (!chunkLoaded((int) Math.floor(p.x) >> 4, (int) Math.floor(p.z) >> 4)) return AIR;
            if (p.y >= 120 || (p.x < 0 || farShore && p.x >= 500) && p.y < 40) return STONE;
            return p.y < 31 ? LAVA : AIR;
        }
    }

    @Test void sixtyFourRocketsCrossFiveHundredBlockSeaAndLandOnNewlyLoadedShore() throws Exception {
        runSea(64, true, true);
    }

    @Test void threeRocketsRejectBeforeLaunchOrPhysicallyReturnWithoutLava() throws Exception {
        runSea(3, true, false);
    }

    @Test void absentFarShoreReservesAndPhysicallyReturnsToLaunchShore() throws Exception {
        runSea(64, false, false);
    }

    private void runSea(int initialRockets, boolean farShore, boolean crossingRequired) throws Exception {
        LavaSea world = new LavaSea(farShore);
        Vector3d start = new Vector3d(-8.5, 40, .5);
        Vector3i goal = new Vector3i(516, 40, 0);
        assertFalse(world.chunkLoaded(500 >> 4, 0), "far shore must initially be unknown");
        assertSame(LAVA, world.getBlockStateAt(new Vector3d(100, 30, 0)));
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.position.set(new Vector3d(start));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        inventory[36] = item("firework_rocket", initialRockets);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        assertFalse(TravelMovement.isOpen(world, start));
        assertTrue(world.isOnGround(start));
        TravelMovement travel = new TravelMovement(goal, movementSync.beginStaticNavigationRequest(goal), true);
        var logger = (ch.qos.logback.classic.Logger) MovementSync.getLogger();
        var messages = new ch.qos.logback.core.read.ListAppender<ch.qos.logback.classic.spi.ILoggingEvent>();
        messages.start();
        logger.addAppender(messages);
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
        boolean crossedSea = false;
        boolean sawFarShoreLoad = false;
        double flightDistance = 0;
        int tick = 0;
        var scheduledMonitors = java.util.Collections.newSetFromMap(
                new java.util.IdentityHashMap<Object, Boolean>());
        try {
            for (; tick < 2400 && !travel.isFinished(); tick++) {
                // Wait for genuine worker results, never replace/publish routes ourselves.
                // This keeps the simulated 20 Hz clock independent of machine throughput.
                world.observe();
                sawFarShoreLoad |= world.chunkLoaded(500 >> 4, 0);
                awaitPlanning(travel);
                Object child = field(travel, "child");
                if (child instanceof ElytraFlyMovement) {
                    awaitPlanning(child);
                    Object monitor = field(child, "reserveMonitor");
                    if (scheduledMonitors.add(monitor)) {
                        var scheduler = monitor.getClass().getDeclaredMethod(
                                "setExecutorForTesting", java.util.concurrent.Executor.class);
                        scheduler.setAccessible(true);
                        // Execute the genuine producer before observe returns to its existing
                        // latest() consumer. No extra onTick, fabricated report, or physics step.
                        scheduler.invoke(monitor, (java.util.concurrent.Executor) Runnable::run);
                    }
                }
                travel.onTick();
                child = field(travel, "child");
                if (child instanceof ElytraFlyMovement) {
                    Object monitor = field(child, "reserveMonitor");
                    var reserveAwait = monitor.getClass().getDeclaredMethod("awaitIdle", long.class, TimeUnit.class);
                    reserveAwait.setAccessible(true);
                    reserveAwait.invoke(monitor, 30L, TimeUnit.SECONDS);
                    Object controller = field(child, "localGoto");
                    var await = controller.getClass().getDeclaredMethod("awaitIdle", long.class, TimeUnit.class);
                    await.setAccessible(true);
                    await.invoke(controller, 30L, TimeUnit.SECONDS);
                    var result = ((xin.bbtt.flight.LocalGotoController) controller).latest();
                    if (result != null && result.status() == xin.bbtt.flight.LocalGotoController.Status.SAFE_PREFIX)
                        sawLocalPrefix = true;
                }
                if (crossingRequired) assertNull(travel.lowFlightFailure(), "flight rejected at tick=" + tick);
                if (deployments == 0 && travel.lowFlightFailure() != null) break;
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
                        assertTrue(rocketRequests <= initialRockets, "cannot consume nonexistent rockets");
                        inventory[36] = rocketRequests == initialRockets ? null
                                : item("firework_rocket", initialRockets - rocketRequests);
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
                assertTrue(p.y >= 31 && p.y + height <= 120 + 1.e-6,
                        "lava/roof contact: " + diagnostic);
                if (p.x < 0 || farShore && p.x >= 500)
                    assertTrue(p.y >= 40, "shore penetration: " + diagnostic);
                assertTrue(next.fallDistance() < 3, "damaging fall: " + diagnostic);
                assertTrue(world.chunkLoaded((int) Math.floor(p.x) >> 4, (int) Math.floor(p.z) >> 4),
                        "entered unknown terrain: " + diagnostic);
                if (!gliding && p.y > start.y + .5) jumped = true;
                if (gliding) {
                    flightTicks++;
                    flightDistance += Math.hypot(p.x - state.position().x, p.z - state.position().z);
                    if (p.x > 1 && p.x < 499) crossedSea = true;
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
            assertEquals(initialRockets - rocketRequests, movementSync.countHotbarFireworks(),
                    "inventory must reflect every real use-item request");
            assertTrue(movementSync.onGround.get(), "must end physically grounded: " + diagnostic);
            assertFalse(movementSync.isGliding(), "must receive landing acknowledgement: " + diagnostic);
            assertEquals(40, movementSync.position.get().y, 1.e-6, diagnostic);
            if (crossingRequired) {
                assertTrue(travel.isFinished(), "crossing must complete: " + diagnostic);
                assertTrue(jumped && deployments > 0 && crossedSea, "must launch and fly over sea: " + diagnostic);
                assertTrue(sawFarShoreLoad, "shore must load from physical approach");
                assertTrue(flightDistance >= 500, "must fly full sea, not walk: " + diagnostic);
                assertTrue(rocketRequests > 0 && groundedAcknowledgements > 0, diagnostic);
                assertTrue(movementSync.position.get().distance(new Vector3d(516.5, 40, .5)) <= 1.5,
                        "physical endpoint must be far shore: " + diagnostic);
                assertTrue(sawLocalPrefix, "must exercise real local planner");
            } else if (deployments == 0) {
                assertEquals(3, initialRockets, "missing shore with 64 rockets must exercise reserved return");
                assertNotNull(travel.lowFlightFailure(), "prelaunch rejection must be explicit");
                assertFalse(jumped, "prelaunch rejection must not jump");
                assertEquals(0, rocketRequests, "rejection must consume no rockets");
                assertTrue(movementSync.position.get().x < 0, diagnostic);
            } else {
                assertTrue(crossedSea && rocketRequests > 0 && groundedAcknowledgements > 0,
                        "return must follow an actual powered flight over lava: " + diagnostic);
                assertTrue(movementSync.position.get().x < 0, "return must land on launch shore: " + diagnostic);
                assertTrue(messages.list.stream().map(e -> e.getFormattedMessage().toLowerCase(java.util.Locale.ROOT))
                                .anyMatch(m -> m.contains("planned-return") || m.contains("planned return")
                                        || m.contains("planned_return")),
                        "physical return requires an explicit production planned-return diagnostic, not inferred failure");
            }
        } finally {
            logger.detachAppender(messages);
            messages.stop();
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
        for (Field candidate : owner.getClass().getDeclaredFields()) {
            candidate.setAccessible(true);
            Object pending = candidate.get(owner);
            if (pending instanceof CompletableFuture<?> future) future.get(150, TimeUnit.SECONDS);
        }
    }
}
