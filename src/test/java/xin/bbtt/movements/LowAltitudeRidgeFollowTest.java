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
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.joml.Vector3d;
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
import xin.bbtt.tasks.updateMotionTask;

import static org.junit.jupiter.api.Assertions.*;

/** Real follow orchestration and physics; only the moving entity and server acknowledgements are simulated. */
final class LowAltitudeRidgeFollowTest extends ElytraCommandTestSupport {
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

    /** Infinite transverse ridge: passing around its ends cannot fake climbing over it. */
    private static final class RidgeWorld extends World {
        private final boolean ceiling;
        RidgeWorld(boolean ceiling) { this.ceiling = ceiling; }
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d p) {
            return p.y < 64 || p.x >= 112 && p.x < 128 && p.y < 84 ? STONE : AIR;
        }
    }

    @Test void followsThroughTwentyHighSixteenWideStoneRidge() throws Exception {
        exercise(false, 720f);
    }

    private void exercise(boolean ceiling, float initialYaw) throws Exception {
        dimensions.set(0, NbtMap.builder().putBoolean("has_ceiling", ceiling)
                .putInt("min_y", 0).putInt("height", 256).putInt("logical_height", ceiling ? 128 : 256).build());
        RidgeWorld world = new RidgeWorld(ceiling);
        Vector3d start = new Vector3d(16.5, 64, 8.5);
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.position.set(new Vector3d(start));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        movementSync.yaw.set(initialYaw);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        Entity entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                36.5, 64, 8.5, 0, 0, 0, new Vector3d());
        entity.getMetadata().put("shared_flags", (byte) 0);
        world.getEntities().put(99, entity);
        FollowMovement follow = new FollowMovement(99, 8, false, movementSync.beginNavigationRequest());
        follow.init();
        PlayerPose pose = PlayerPose.STANDING;
        int deployments = 0;
        int rocketRequests = 0;
        int rocketTicks = 0;
        int flightTicks = 0;
        int groundedAcknowledgements = 0;
        boolean jumped = false;
        boolean crossedRidge = false;
        boolean targetInsideRidge = false;
        int movingFlightTicks = 0;
        double flightDistance = 0;
        int tick = 0;
        int firstDeploymentTick = -1;
        updateMotionTask wire = new updateMotionTask();
        List<Float> wireYaws = new ArrayList<>();
        // Seed the server-visible orientation, including accumulated turns.
        wire.syncPositionToServer(false, true, true);
        wireYaws.add(assertInstanceOf(ServerboundMovePlayerPosRotPacket.class,
                packets.get(packets.size() - 1)).getYaw());
        assertEquals(initialYaw, wireYaws.get(0).floatValue());
        try {
            for (; tick < 1000; tick++) {
                // 0.8 blocks/tick outruns walking but is comfortably below rocket flight speed.
                // Only the target is scripted. The bot's position is never corrected.
                if (tick < 300) {
                    double x = 36.5 + (tick + 1) * .8;
                    entity.move(new Vector3d(x - entity.getPosition().x, 0, 0));
                }
                if (entity.getPosition().x >= 112 && entity.getPosition().x < 128) targetInsideRidge = true;
                // Wait for genuine worker results, never replace/publish routes ourselves.
                // This keeps the simulated 20 Hz clock independent of machine throughput.
                awaitPlanning(follow);
                Object child = field(follow, "child");
                if (child instanceof ElytraFlyMovement) awaitPlanning(child);
                follow.onTick();
                var input = movementSync.snapshotInput();
                var packet = movementSync.pollBeforeMovementPacket();
                while (packet != null) {
                    Bot.INSTANCE.getSession().send(packet);
                    if (packet instanceof ServerboundPlayerCommandPacket command
                            && command.getState() == PlayerState.START_ELYTRA_FLYING) {
                        assertFalse(movementSync.onGround.get(), "deployment requires the real jump");
                        assertTrue(movementSync.velocity.get().y <= 0, "deploy on descent");
                        deployments++;
                        if (firstDeploymentTick < 0) firstDeploymentTick = tick;
                        assertEquals(1, deployments, "continuous follow must never redeploy: tick=" + tick);
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
                PhysicsState state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                        movementSync.onGround.get(), movementSync.fallDistance.get(), pose);
                double height = (double) (float) pose.height();
                PhysicsResult next = gliding
                        ? new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(),
                                movementSync.hasAttachedFirework(), input, movementSync.getMovementParameters())
                                .step(world, state, HALF_WIDTH, height)
                        : new WalkingPhysics(input, movementSync.yaw.get(), true, movementSync.getMovementParameters())
                                .step(world, state, HALF_WIDTH, height);
                String diagnostic = "tick=" + tick + " position=" + next.position() + " target=" + entity.getPosition();
                assertFalse(next.collidedX() || next.collidedZ(), "horizontal impact: " + diagnostic);
                Vector3d p = next.position();
                assertTrue(p.isFinite(), diagnostic);
                if (p.x >= 112 && p.x < 128) {
                    assertTrue(p.y >= 84, "must clear the ridge: " + diagnostic);
                    crossedRidge = true;
                }
                assertTrue(p.y >= 64 && (!ceiling || p.y + height <= 80 + 1.e-6),
                        "floor/roof penetration: " + diagnostic);
                assertTrue(world.chunkLoaded((int) Math.floor(p.x) >> 4, (int) Math.floor(p.z) >> 4),
                        "entered unknown terrain: " + diagnostic);
                if (!gliding && p.y > start.y + .5) jumped = true;
                if (gliding) {
                    flightTicks++;
                    flightDistance += Math.hypot(p.x - state.position().x, p.z - state.position().z);
                    if (tick < 300) movingFlightTicks++;
                }
                if (tick >= 60 && tick < 300) {
                    // Bound real pursuit lag, including the climb while the target crosses solid stone.
                    assertTrue(p.distance(entity.getPosition()) <= 40, "pursuit fell behind: " + diagnostic);
                }
                if (deployments > 0 && tick < 300) {
                    assertTrue(gliding, "must keep following in flight while target moves: " + diagnostic);
                    assertFalse(next.onGround(), "no touchdown after deployment during target movement: " + diagnostic);
                }
                assertFalse(follow.isFinished(), "live follow must stay active: " + diagnostic);
                // Every position after setup comes exclusively from the shared physics step.
                movementSync.position.set(p);
                movementSync.velocity.set(next.velocity());
                movementSync.onGround.set(next.onGround());
                movementSync.fallDistance.set(next.fallDistance());
                // Use the real movement packet writer, not a synthetic yaw packet or normalized angle.
                wire.syncPositionToServer(next.collidedX() || next.collidedZ(), true, true);
                var movePacket = assertInstanceOf(ServerboundMovePlayerPosRotPacket.class,
                        packets.get(packets.size() - 1));
                assertEquals(p.x, movePacket.getX(), 0, diagnostic);
                assertEquals(p.y, movePacket.getY(), 0, diagnostic);
                assertEquals(p.z, movePacket.getZ(), 0, diagnostic);
                float previousYaw = wireYaws.get(wireYaws.size() - 1);
                float yaw = movePacket.getYaw();
                assertTrue(Float.isFinite(yaw), "non-finite wire yaw: " + diagnostic);
                double delta = Math.abs((double) yaw - previousYaw);
                assertTrue(delta <= 180, "wire yaw must choose nearest continuous equivalent: previous="
                        + previousYaw + " yaw=" + yaw + " " + diagnostic);
                assertFalse(delta > 320 && Math.abs(yaw) < 360,
                        "Grim modulo-wrap signature: previous=" + previousYaw + " yaw=" + yaw + " " + diagnostic);
                wireYaws.add(yaw);
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
            assertFalse(follow.isFinished(), "a live follow must remain active: " + diagnostic);
            assertTrue(targetInsideRidge, "scripted target must temporarily be inside stone");
            assertTrue(crossedRidge, "bot must physically cross above the ridge");
            assertTrue(jumped, "ground launch must actually jump: " + diagnostic);
            assertEquals(1, deployments, "exactly one deployment for the entire follow: " + diagnostic);
            assertTrue(rocketRequests > 0, "must exercise real rocket requests: " + diagnostic);
            assertTrue(packets.stream().anyMatch(ServerboundUseItemPacket.class::isInstance));
            assertTrue(firstDeploymentTick >= 0 && firstDeploymentTick < 100,
                    "must launch promptly, not wait for target to stop: " + diagnostic);
            assertEquals(300 - firstDeploymentTick, movingFlightTicks,
                    "every moving-target tick after deployment must remain in flight: " + diagnostic);
            assertTrue(flightDistance >= 200, "must fly the pursuit rather than walk it: " + diagnostic);
            assertEquals(tick + 1, wireYaws.size(), "wire yaw must be checked through launch, pursuit and landing");
            assertEquals(276.5, entity.getPosition().x, 1.e-6, "target travels +X for exactly 15 seconds");
            assertTrue(groundedAcknowledgements > 0, "stopped target must lead to physical landing");
            assertTrue(movementSync.onGround.get(), diagnostic);
            assertFalse(movementSync.isGliding(), diagnostic);
            assertEquals(64, movementSync.position.get().y, 1.e-6, diagnostic);
            assertTrue(movementSync.position.get().distance(entity.getPosition()) <= 8,
                    "must settle near concrete stopped endpoint (276.5,64,8.5) within follow8: " + diagnostic);
        } finally {
            follow.onStop();
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
