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
final class LowAltitudeTravelEndToEndTest extends ElytraCommandTestSupport {
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

    /** Voxel geometry is shared by planning, clipping, support and excluded-support queries. */
    private static final class Corridor extends World {
        private final boolean bent;
        private final boolean rolling;
        private volatile int loadedCenterZ;
        private int loadingChanges;

        Corridor(boolean bent, boolean rolling) {
            this.bent = bent;
            this.rolling = rolling;
        }

        @Override public boolean chunkLoaded(int x, int z) {
            return !rolling || x >= -2 && x <= 2 && Math.abs(z - loadedCenterZ) <= 3;
        }

        @Override public BlockState getBlockStateAt(Vector3d p) {
            boolean corridor = !bent || p.x >= 0 && p.x < 32 && p.z >= 0 && p.z < 112
                    || p.x >= 0 && p.x < 112 && p.z >= 80 && p.z < 112;
            return p.y < 64 || p.y >= 80 || !corridor ? STONE : AIR;
        }

        void loadAround(Vector3d position) {
            int next = (int) Math.floor(position.z) >> 4;
            if (rolling && next != loadedCenterZ) {
                loadedCenterZ = next;
                loadingChanges++;
            }
        }
    }

    @Test
    void groundLaunchUnderRoofUsesRocketAndPhysicallyLandsAtGoal() throws Exception {
        exercise(new Corridor(false, false), new Vector3d(16.5, 64, 8.5),
                new Vector3i(16, 64, 136), false);
    }

    @Test
    void flightTurnsThroughBentRoofedCorridorWithoutCuttingSolidCorner() throws Exception {
        exercise(new Corridor(true, false), new Vector3d(16.5, 64, 8.5),
                new Vector3i(100, 64, 96), false);
    }

    @Test
    void rollingLoadedSectionsAndVelocityPerturbationStillReachPhysicalEndpoint() throws Exception {
        Corridor world = new Corridor(false, true);
        Vector3i goal = new Vector3i(16, 64, 216);
        assertFalse(world.chunkLoaded(goal.x >> 4, goal.z >> 4), "goal starts outside loaded terrain");
        exercise(world, new Vector3d(16.5, 64, 8.5), goal, true);
        assertTrue(world.loadingChanges >= 6, "flight must cross multiple streamed sections");
        assertFalse(world.chunkLoaded(1, 0), "old sections really unload, not merely accumulate");
    }

    @Test
    void sixHundredBlockUnderRoofTravelLaunchesBeforeGoalLoadsAndArrivesIn3d() throws Exception {
        Corridor world = new Corridor(false, true);
        Vector3d start = new Vector3d(16.5, 64, 8.5);
        Vector3i goal = new Vector3i(16, 64, 608);
        assertFalse(world.chunkLoaded(goal.x >> 4, goal.z >> 4));
        var durability = movementSync.inventoryManager.getElytraDurability();
        assertTrue(durability.usable(), "fixture must equip usable elytra");
        assertTrue(durability.canStart(600), "600-block flight must not be rejected by durability budget");
        exercise(world, start, goal, false, true);
        assertTrue(world.loadingChanges >= 30, "actual movement must stream terrain along the 600-block route");
        assertTrue(world.chunkLoaded(goal.x >> 4, goal.z >> 4), "destination eventually streams in");
        assertFalse(world.chunkLoaded(1, 0), "old terrain must unload");
    }

    private void exercise(Corridor world, Vector3d start, Vector3i goal, boolean perturb) throws Exception {
        exercise(world, start, goal, perturb, false);
    }

    private void exercise(Corridor world, Vector3d start, Vector3i goal, boolean perturb,
                          boolean requirePartialLaunch) throws Exception {
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
        boolean perturbed = false;
        boolean jumped = false;
        boolean roundedCorner = false;
        double flightDistance = 0;
        int tick = 0;
        try {
            for (; tick < 1600 && !travel.isFinished(); tick++) {
                world.loadAround(movementSync.position.get());
                // Wait for genuine worker results, never replace/publish routes ourselves.
                // This keeps the simulated 20 Hz clock independent of machine throughput.
                awaitPlanning(travel);
                Object child = field(travel, "child");
                if (child instanceof ElytraFlyMovement) awaitPlanning(child);
                travel.onTick();
                if (requirePartialLaunch) {
                    assertNull(travel.lowFlightFailure(), "travel rejected flight at tick=" + tick
                            + " position=" + movementSync.position.get());
                    if (movementSync.getActiveGoal() == null) {
                        assertTrue(movementSync.onGround.get() && !movementSync.isGliding()
                                        && movementSync.position.get().distance(
                                                new Vector3d(goal.x + .5, goal.y, goal.z + .5)) <= 1.5,
                                "navigation cleared before supported 3D arrival at tick=" + tick
                                        + " position=" + movementSync.position.get());
                    }
                    if (tick == 100) assertTrue(deployments > 0,
                            "must launch promptly instead of waiting for the unloaded destination");
                }
                var input = movementSync.snapshotInput();
                var packet = movementSync.pollBeforeMovementPacket();
                while (packet != null) {
                    assertEquals(MovementSync.BeforeMovementDispatch.SENT,
                            movementSync.sendBeforeMovementPacket(packet, packet));
                    if (packet instanceof ServerboundPlayerCommandPacket command
                            && command.getState() == PlayerState.START_ELYTRA_FLYING) {
                        assertFalse(movementSync.onGround.get(), "deployment requires the real jump");
                        assertTrue(movementSync.velocity.get().y <= 0, "deploy on descent");
                        if (requirePartialLaunch && deployments == 0) {
                            assertFalse(world.chunkLoaded(goal.x >> 4, goal.z >> 4),
                                    "first deployment must use a partial route before the destination loads");
                        }
                        deployments++;
                        sharedFlags(true);
                    } else if (packet instanceof ServerboundUseItemPacket) {
                        assertTrue(movementSync.isGliding(), "rocket request must follow deployment");
                        assertEquals(0, rocketTicks, "do not stack attached rockets");
                        assertTrue(movementSync.countHotbarFireworks() > 0);
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
                if (perturb && !perturbed && gliding && flightTicks >= 25) {
                    // A velocity impulse, not a position correction or artificial steering aid.
                    movementSync.velocity.set(new Vector3d(movementSync.velocity.get()).add(.12, 0, -.08));
                    perturbed = true;
                }
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
                assertTrue(p.y >= 64 && p.y + height <= 80 + 1.e-6, "floor/roof penetration: " + diagnostic);
                assertTrue(world.chunkLoaded((int) Math.floor(p.x) >> 4, (int) Math.floor(p.z) >> 4),
                        "entered unknown terrain: " + diagnostic);
                if (!gliding && p.y > start.y + .5) jumped = true;
                if (gliding) {
                    flightTicks++;
                    flightDistance += Math.hypot(p.x - state.position().x, p.z - state.position().z);
                    if (world.bent && p.x > 32) {
                        assertTrue(p.z >= 80 + HALF_WIDTH, "must fly around, not through, the corner");
                        roundedCorner = true;
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
                    assertTrue(p.distance(new Vector3d(goal.x + .5, goal.y, goal.z + .5)) <= 1.5,
                            "flight leg touched down before the physical endpoint: " + diagnostic
                                    + " before=" + state + " resultVelocity=" + next.velocity()
                                    + " yaw=" + movementSync.yaw.get() + " pitch=" + movementSync.pitch.get()
                                    + " attached=" + movementSync.hasAttachedFirework()
                                    + " admissionFailure=" + movementSync.getFlightControlFailure());
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
            if (world.bent) assertTrue(roundedCorner, "bend must be traversed in flight");
            if (perturb) assertTrue(perturbed, "must exercise perturbation during active flight");
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
