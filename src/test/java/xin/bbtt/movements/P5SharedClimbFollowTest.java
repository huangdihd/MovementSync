package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.Map;
import java.util.OptionalInt;
import java.util.UUID;
import java.util.concurrent.TimeUnit;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Entity.Entity;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.flight.LocalClimbHandoff;
import xin.bbtt.flight.LocalGotoController;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** Fresh open-air follow shares clearance geometry without surrendering its entity owner. */
class P5SharedClimbFollowTest extends ElytraCommandTestSupport {
    @Test void freshFollowClimbsThenContinuesLocalEntityPursuit() throws Exception {
        var stone = new BlockState("stone", 1, Map.of(), "block", 1.5, true, "STONE");
        var air = new BlockState("air", 0, Map.of(), "empty", 0, false, "AIR");
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 64 ? stone : air; }
        };
        Field worldField = movementSync.getClass().getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.movementController.pause();
        movementSync.setRecoveryDimension("minecraft:overworld");
        // The independent follow switch also owns its shared deployment climb.
        movementSync.getFlightScopeRouter().setLocalGotoEnabled(false);
        movementSync.position.set(new Vector3d(.5, 64, .5));
        movementSync.velocity.set(new Vector3d(0, -.0784, 0));
        movementSync.onGround.set(true);
        movementSync.setGliding(false);
        movementSync.yaw.set(-90f);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        Entity entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                40, 66, .5, 0, 0, 0, new Vector3d());
        entity.getMetadata().put("shared_flags", (byte) 0x80);
        world.getEntities().put(99, entity);
        var target = new EntityFlightTarget(99, 6, entity.getPosition());
        var owner = new ElytraFlyMovement(target, movementSync.beginNavigationRequest(), 6).forFollow();
        int rockets = 0, rocketTicks = 0, deployments = 0, cruiseTicks = 0;
        boolean climbed = false;
        try {
            owner.init();
            var handoff = (LocalClimbHandoff) field(owner, "climbHandoff");
            var provider = (LocalGotoController) field(owner, "localGoto");
            for (int tick = 0; tick < 180 && cruiseTicks < 45; tick++) {
                entity.moveTo(new Vector3d(40 + .6 * tick, 66, .5));
                target.sample(world);
                owner.onTick();
                assertTrue(provider.awaitIdle(30, TimeUnit.SECONDS));
                climbed |= handoff.phase() == LocalClimbHandoff.Phase.LOCAL_CLIMB;
                if (handoff.phase() == LocalClimbHandoff.Phase.CRUISE) {
                    assertTrue(climbed, "must not inherit airborne cruise on fresh deployment");
                    cruiseTicks++;
                    assertNotNull(provider.latest());
                    assertNotEquals(LocalGotoController.Status.LANDED, provider.latest().status());
                    var enabled = ElytraFlyMovement.class.getDeclaredMethod("localFollowEnabled", xin.bbtt.MovementSync.class);
                    enabled.setAccessible(true);
                    assertEquals(true, enabled.invoke(owner, movementSync));
                }
                assertSame(target, field(owner, "target"), "never replace entity with the clearance waypoint");
                assertFalse(owner.isFinished());
                var input = movementSync.snapshotInput();
                Object packet;
                while ((packet = movementSync.pollBeforeMovementPacket()) != null) {
                    if (packet instanceof ServerboundPlayerCommandPacket) {
                        deployments++;
                        assertFalse(movementSync.onGround.get());
                        movementSync.applySharedFlags((byte) 0x80);
                    } else if (packet instanceof ServerboundUseItemPacket) {
                        assertEquals(0, rocketTicks);
                        inventory[36] = item("firework_rocket", 64 - ++rockets);
                        movementSync.inventoryManager.setContainerItems(0, inventory);
                        var rocket = new Entity(9001, new UUID(0, 9001), EntityType.FIREWORK_ROCKET,
                                0, 0, 0, 0, 0, 0, new Vector3d());
                        rocket.getMetadata().put("attached_to_target", OptionalInt.of(movementSync.entityId));
                        world.getEntities().put(9001, rocket);
                        rocketTicks = 30;
                    }
                }
                boolean gliding = movementSync.isGliding();
                var pose = gliding ? PlayerPose.GLIDING : PlayerPose.STANDING;
                var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                        movementSync.onGround.get(), movementSync.fallDistance.get(), pose);
                var next = gliding ? new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(),
                        movementSync.hasAttachedFirework(), input, movementSync.getMovementParameters())
                        .step(world, state, PlayerPose.HALF_WIDTH, pose.height())
                        : new WalkingPhysics(input, movementSync.yaw.get(), true, movementSync.getMovementParameters())
                        .step(world, state, PlayerPose.HALF_WIDTH, pose.height());
                assertFalse(next.collidedX() || next.collidedZ());
                if (gliding) assertFalse(next.onGround(), "deployment must not settle back to ground");
                movementSync.position.set(next.position());
                movementSync.velocity.set(next.velocity());
                movementSync.onGround.set(next.onGround());
                movementSync.fallDistance.set(next.fallDistance());
                movementSync.completePhysicsStep();
                if (rocketTicks > 0 && --rocketTicks == 0) world.getEntities().remove(9001);
            }
            assertTrue(climbed);
            assertEquals(45, cruiseTicks, "clearance must finish despite same-height moving target");
            assertEquals(1, deployments);
            assertTrue(rockets > 0);
        } finally { owner.onStop(); world.getEntities().remove(9001); }
    }

    private static Object field(Object owner, String name) throws Exception {
        Field field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(owner);
    }
}
