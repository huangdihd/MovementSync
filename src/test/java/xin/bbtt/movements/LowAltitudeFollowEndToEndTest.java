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

/** Real follow orchestration and physics; only the moving entity and server acknowledgements are simulated. */
final class LowAltitudeFollowEndToEndTest extends ElytraCommandTestSupport {
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

    /** Broad hill: one block per eight horizontal blocks, then the same descent. */
    private static final class Hill extends World {
        private final boolean ceiling;
        Hill(boolean ceiling) { this.ceiling = ceiling; }
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        static int floor(double z) {
            if (z < 64 || z >= 224) return 64;
            if (z < 128) return 64 + (int) ((z - 64) / 8);
            if (z < 160) return 72;
            return 71 - (int) ((z - 160) / 8);
        }
        @Override public BlockState getBlockStateAt(Vector3d p) {
            return p.y < floor(p.z) || ceiling && p.y >= 80 ? STONE : AIR;
        }
    }

    @Test void followsGroundTargetUphillAndDownhillUnderNetherRoof() throws Exception {
        exercise(true);
    }

    @Test void followsGroundTargetUphillAndDownhillInOpenOverworld() throws Exception {
        exercise(false);
    }

    private void exercise(boolean ceiling) throws Exception {
        dimensions.set(0, NbtMap.builder().putBoolean("has_ceiling", ceiling)
                .putInt("min_y", 0).putInt("height", 256).putInt("logical_height", ceiling ? 128 : 256).build());
        Hill world = new Hill(ceiling);
        Vector3d start = new Vector3d(16.5, 64, 8.5);
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.position.set(new Vector3d(start));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        Entity entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                16.5, 64, 28.5, 0, 0, 0, new Vector3d());
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
        double uphillHeight = Double.NEGATIVE_INFINITY;
        double downhillHeight = Double.POSITIVE_INFINITY;
        int movingFlightTicks = 0;
        double flightDistance = 0;
        int tick = 0;
        try {
            for (; tick < 1000; tick++) {
                // 0.8 blocks/tick outruns walking but is comfortably below rocket flight speed.
                // Only the target is scripted. The bot's position is never corrected.
                if (tick < 400) {
                    double z = 28.5 + (tick + 1) * .8;
                    entity.move(new Vector3d(0, Hill.floor(z) - entity.getPosition().y,
                            z - entity.getPosition().z));
                }
                // Wait for genuine worker results, never replace/publish routes ourselves.
                // This keeps the simulated 20 Hz clock independent of machine throughput.
                awaitPlanning(follow);
                Object child = field(follow, "child");
                if (child instanceof ElytraFlyMovement) awaitPlanning(child);
                follow.onTick();
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
                assertTrue(p.y >= Hill.floor(p.z) && (!ceiling || p.y + height <= 80 + 1.e-6),
                        "floor/roof penetration: " + diagnostic);
                assertTrue(world.chunkLoaded((int) Math.floor(p.x) >> 4, (int) Math.floor(p.z) >> 4),
                        "entered unknown terrain: " + diagnostic);
                if (!gliding && p.y > start.y + .5) jumped = true;
                if (gliding) {
                    flightTicks++;
                    flightDistance += Math.hypot(p.x - state.position().x, p.z - state.position().z);
                    if (tick < 400) movingFlightTicks++;
                    if (p.z >= 128 && p.z < 160) uphillHeight = Math.max(uphillHeight, p.y);
                    if (p.z >= 256 && tick < 400) downhillHeight = Math.min(downhillHeight, p.y);
                }
                if (tick >= 60 && tick < 400) {
                    // Eight-block requested offset plus < three seconds of target travel
                    // (48 blocks) and launch/turn margin; this rejects unbounded lag.
                    assertTrue(p.distance(entity.getPosition()) <= 64, "pursuit fell behind: " + diagnostic);
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
            assertFalse(follow.isFinished(), "a live follow must remain active: " + diagnostic);
            assertTrue(jumped, "ground launch must actually jump: " + diagnostic);
            assertTrue(deployments > 0, "walking fallback is not a flight E2E: " + diagnostic);
            assertTrue(rocketRequests > 0, "must exercise real rocket requests: " + diagnostic);
            assertTrue(packets.stream().anyMatch(ServerboundUseItemPacket.class::isInstance));
            assertTrue(movingFlightTicks >= 200, "must pursue in flight, not repeatedly land and walk: " + diagnostic);
            assertTrue(flightDistance >= 256, "must fly across both slopes: " + diagnostic);
            assertTrue(uphillHeight >= 72, "must physically rise above the hill: " + uphillHeight);
            assertTrue(Double.isFinite(downhillHeight) && downhillHeight <= uphillHeight - 3,
                    "must descend after the hill, not retain peak altitude: peak=" + uphillHeight + " descent=" + downhillHeight);
            assertTrue(groundedAcknowledgements > 0, "stopped target must lead to physical landing");
            assertTrue(movementSync.onGround.get(), diagnostic);
            assertFalse(movementSync.isGliding(), diagnostic);
            assertEquals(64, movementSync.position.get().y, 1.e-6, diagnostic);
            assertTrue(movementSync.position.get().distance(entity.getPosition()) <= 12,
                    "must settle near concrete stopped endpoint (16.5,64,348.5): " + diagnostic);
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
