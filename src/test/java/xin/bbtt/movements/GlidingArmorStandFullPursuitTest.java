package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.Map;
import java.util.OptionalInt;
import java.util.UUID;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.EntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.MetadataTypes;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.type.ByteEntityMetadata;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundMoveEntityPosPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundSetEntityDataPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Entity.Entity;
import xin.bbtt.Entity.EntityRegistry;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.listeners.EntityPacketListener;
import xin.bbtt.mcbot.events.ReceivePacketEvent;
import xin.bbtt.physics.EntityFlightTarget;
import xin.bbtt.physics.GlidingPhysics;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Full 237-block packet-driven pursuit, not merely a sampled target/state test. */
final class GlidingArmorStandFullPursuitTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");

    @Test void followsGlidingArmorStandFrom4014To4251AtSixteenBlocksPerSecond() throws Exception {
        movementSync.getMovementController().pause();
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 32 ? STONE : AIR; }
        };
        Field worldField = movementSync.getClass().getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        // Start in acknowledged flight; subsequent bot positions come only from real physics.
        movementSync.position.set(new Vector3d(4008, 40, .5));
        movementSync.velocity.set(new Vector3d(.8, 0, 0));
        movementSync.onGround.set(false);
        movementSync.yaw.set(-90f);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        Entity entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                4014, 40, .5, 0, 0, 0, new Vector3d());
        world.getEntities().put(99, entity);
        EntityPacketListener listener = new EntityPacketListener();
        sharedFlags(listener, 99, "ARMOR_STAND");
        sharedFlags(listener, movementSync.entityId, "PLAYER");
        assertTrue(movementSync.isGliding());
        FollowMovement follow = new FollowMovement(99, 6, false, movementSync.beginNavigationRequest());
        follow.init();
        int rockets = 0, rocketTicks = 0, updates = 0;
        double distance = 0;
        try {
            // 296 full 0.8-block ticks plus the final 0.2-block tick reach exactly 4251.
            // Then keep observing the still-present, still-gliding target for >7 seconds.
            for (int tick = 0; tick < 457; tick++) {
                if (tick < 297) {
                    double x = Math.min(4251, 4014 + (tick + 1) * .8);
                    var packet = new ClientboundMoveEntityPosPacket(99, x - entity.getPosition().x, 0, 0, false);
                    listener.OnMoveEntityPosition(new ReceivePacketEvent<>(packet, ClientboundMoveEntityPosPacket.class));
                    assertEquals(x, entity.getPosition().x, 1.e-8);
                    updates++;
                }
                follow.onTick();
                String context = "tick=" + tick + " bot=" + movementSync.position.get() + " target=" + entity.getPosition();
                EntityFlightTarget target = (EntityFlightTarget) field(follow, "target");
                assertFalse(target.lost(), "present world entity is never lost: " + context);
                assertTrue(target.gliding(), "armor stand shared_flags must retain 0x80: " + context);
                assertFalse(target.landed(), context);
                assertFalse((boolean) field(follow, "reportedLost"), context);
                assertFalse((boolean) field(follow, "landing"), "must not land beneath a gliding target: " + context);
                assertFalse(follow.isFinished(), context);
                assertInstanceOf(ElytraFlyMovement.class, field(follow, "child"), context);
                var request = movementSync.pollBeforeMovementPacket();
                while (request != null) {
                    assertEquals(xin.bbtt.MovementSync.BeforeMovementDispatch.SENT,
                            movementSync.sendBeforeMovementPacket(request, request));
                    assertFalse(request instanceof ServerboundPlayerCommandPacket, "acknowledged flight must not redeploy: " + context);
                    if (request instanceof ServerboundUseItemPacket) {
                        assertEquals(0, rocketTicks, "attached rockets must not stack: " + context);
                        inventory[36] = item("firework_rocket", 64 - ++rockets);
                        movementSync.inventoryManager.setContainerItems(0, inventory);
                        Entity rocket = new Entity(9001, new UUID(0, 9001), EntityType.FIREWORK_ROCKET,
                                movementSync.position.get().x, movementSync.position.get().y, .5,
                                0, 0, 0, new Vector3d());
                        rocket.getMetadata().put("attached_to_target", OptionalInt.of(movementSync.entityId));
                        world.getEntities().put(9001, rocket);
                        rocketTicks = 30;
                    }
                    request = movementSync.pollBeforeMovementPacket();
                }
                assertTrue(movementSync.isGliding(), context);
                var input = movementSync.snapshotInput();
                var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                        movementSync.onGround.get(), movementSync.fallDistance.get(), PlayerPose.GLIDING);
                var next = new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(),
                        movementSync.hasAttachedFirework(), input, movementSync.getMovementParameters())
                        .step(world, state, (double) .6f / 2, (double) .6f);
                assertTrue(next.position().isFinite(), context);
                assertFalse(next.onGround(), "no premature physical touchdown: " + context);
                assertFalse(next.collidedX() || next.collidedZ(), context);
                // Open-sky policy may climb above the target; measure pursuit lag horizontally.
                assertTrue(Math.hypot(next.position().x - entity.getPosition().x,
                        next.position().z - entity.getPosition().z) < 40, "bounded horizontal pursuit lag: " + context);
                distance += next.position().distance(state.position());
                movementSync.position.set(next.position());
                movementSync.velocity.set(next.velocity());
                movementSync.onGround.set(next.onGround());
                movementSync.fallDistance.set(next.fallDistance());
                movementSync.completePhysicsStep();
                if (rocketTicks > 0 && --rocketTicks == 0) world.getEntities().remove(9001);
            }
            assertEquals(297, updates);
            assertEquals(new Vector3d(4251, 40, .5), entity.getPosition());
            assertTrue(distance >= 237, "bot must physically fly the full pursuit");
            assertTrue(movementSync.position.get().x > 4230, "bot must reach the endpoint neighborhood");
            assertTrue(rockets > 0, "exercise real firework requests and acknowledged attachment");
        } finally {
            follow.onStop();
            world.getEntities().remove(9001);
        }
    }

    private static void sharedFlags(EntityPacketListener listener, int id, String type) {
        int index = EntityRegistry.Instance.getEntity(type).getMetadataKeys().indexOf("shared_flags");
        assertTrue(index >= 0);
        var packet = new ClientboundSetEntityDataPacket(id, new EntityMetadata<?, ?>[]{
                new ByteEntityMetadata(index, MetadataTypes.BYTE, (byte) 0x80)});
        listener.OnSetEntityMetadata(new ReceivePacketEvent<>(packet, ClientboundSetEntityDataPacket.class));
    }

    private static Object field(Object owner, String name) throws Exception {
        Field field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(owner);
    }
}
