package xin.bbtt.movements;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.Entity.Entity;
import xin.bbtt.world.World;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import java.util.UUID;
import static org.junit.jupiter.api.Assertions.*;

class FollowMovementTest extends ElytraCommandTestSupport {
    private Entity installTarget(boolean gliding) throws Exception {
        var field = movementSync.getClass().getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) { return false; }
            @Override public boolean isOnGround(Vector3d p) { return false; }
        });
        Entity entity = new Entity(99, UUID.randomUUID(), EntityType.ARMOR_STAND,
                20, 40, 0, 0, 0, 0, new Vector3d());
        entity.getMetadata().put("shared_flags", (byte)(gliding ? 0x80 : 0));
        movementSync.getWorld().getEntities().put(99, entity);
        movementSync.onGround.set(true);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        return entity;
    }
    private Object child(FollowMovement follow) throws Exception {
        var field = FollowMovement.class.getDeclaredField("child");
        field.setAccessible(true);
        return field.get(follow);
    }
    @Test void waterLandingReturnsToWalkingWhileGlidingFlagRemainsSet() throws Exception {
        var entity = installTarget(true);
        movementSync.setGliding(true);
        movementSync.onGround.set(false);
        var follow = new FollowMovement(99, 8, false, movementSync.beginNavigationRequest());
        follow.init(); follow.onTick();
        var water = java.util.stream.IntStream.range(0, xin.bbtt.Block.BlockStateParser.getBlockStateRegistrySize())
                .mapToObj(xin.bbtt.Block.BlockStateParser.Instance::parseStateId)
                .filter(block -> block.blockName().equals("water") && "0".equals(block.getProperty("level")))
                .findFirst().orElseThrow();
        World pool = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d p) { return water; }
        };
        pool.getEntities().put(99, entity);
        var worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true); worldField.set(movementSync, pool);
        follow.onTick();
        var flying = FollowMovement.class.getDeclaredField("flying");
        flying.setAccessible(true);
        assertFalse(flying.getBoolean(follow));
        assertTrue(movementSync.isGliding());
        follow.onTick();
        assertFalse(flying.getBoolean(follow), "water must not redeploy the persistent gliding flag");
        assertFalse(child(follow) instanceof ElytraFlyMovement);
    }

    @Test void glidingTargetLaunchesWithEquipment() throws Exception {
        installTarget(true);
        var follow = new FollowMovement(99, 8, false, movementSync.beginNavigationRequest());
        follow.init(); follow.onTick();
        assertInstanceOf(ElytraFlyMovement.class, child(follow));
    }
    @Test void sustainedFastTargetLaunches() throws Exception {
        var entity = installTarget(false);
        var follow = new FollowMovement(99, 8, false, movementSync.beginNavigationRequest());
        follow.init();
        for (int i = 0; i < 10; i++) {
            entity.move(new Vector3d(.8, 0, 0));
            follow.onTick();
        }
        assertInstanceOf(ElytraFlyMovement.class, child(follow));
    }
    @Test void missingEquipmentPreventsLaunch() throws Exception {
        for (int slot : new int[]{6, 36}) {
            installTarget(true);
            inventory[slot] = null;
            movementSync.inventoryManager.setContainerItems(0, inventory);
            var follow = new FollowMovement(99, 8, false, movementSync.beginNavigationRequest());
            follow.init(); follow.onTick();
            assertFalse(child(follow) instanceof ElytraFlyMovement);
        }
    }
    @Test void insufficientDurabilityPreventsLaunch() throws Exception {
        installTarget(true);
        var patch = new org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponents(new java.util.HashMap<>());
        patch.put(org.geysermc.mcprotocollib.protocol.data.game.item.component.DataComponentTypes.DAMAGE, 431);
        inventory[6] = new org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack(
                xin.bbtt.inventory.ItemRegistry.Instance.getItem("elytra").getId(), 1, patch);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        var follow = new FollowMovement(99, 8, false, movementSync.beginNavigationRequest());
        follow.init(); follow.onTick();
        assertFalse(child(follow) instanceof ElytraFlyMovement);
    }

    @Test void slowTargetNeverLaunchesAndFailedWalkingPlanIsNotRetried() throws Exception {
        var entity = installTarget(false);
        var follow = new FollowMovement(99, 8, false, movementSync.beginNavigationRequest());
        follow.init();
        for (int i = 0; i < 100; i++) {
            entity.move(new Vector3d(.1, 0, 0));
            follow.onTick();
            assertFalse(child(follow) instanceof ElytraFlyMovement);
        }
        var attempted = FollowMovement.class.getDeclaredField("walkingPlanAttempted");
        attempted.setAccessible(true);
        assertTrue(attempted.getBoolean(follow));
    }

    @Test void lostTargetRetainsLastDestinationUntilTouchdownThenFinishes() throws Exception {
        var entity = installTarget(true);
        movementSync.setGliding(true);
        movementSync.onGround.set(false);
        var follow = new FollowMovement(99, 8, false, movementSync.beginNavigationRequest());
        follow.init(); follow.onTick();
        Vector3d last = new Vector3d(entity.getPosition());
        movementSync.getWorld().getEntities().remove(99);
        follow.onTick();
        assertEquals(last, ((ElytraFlyMovement) child(follow)).getTarget());
        assertFalse(follow.isFinished());
        // Simulate completion of the flight child and authoritative landing metadata.
        var field = FollowMovement.class.getDeclaredField("child");
        field.setAccessible(true); field.set(follow, null);
        movementSync.onGround.set(true);
        movementSync.setGliding(false);
        follow.onTick();
        assertTrue(follow.isFinished());
    }

    @Test void landedTargetSelectsGroundAtTargetAndReturnsToWalking() throws Exception {
        var entity = installTarget(true);
        var entities = movementSync.getWorld().getEntities();
        var worldField = movementSync.getClass().getDeclaredField("world");
        worldField.setAccessible(true);
        World floor = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public boolean isBoxColliding(double x, double y, double z, double X, double Y, double Z) { return false; }
            @Override public boolean isOnGround(Vector3d p) { return p.y == 0; }
            @Override public java.util.OptionalDouble findHighestCollisionTop(double x, double y, double z, double X, double Y, double Z) {
                return java.util.OptionalDouble.of(0);
            }
        };
        floor.getEntities().putAll(entities); worldField.set(movementSync, floor);
        movementSync.setGliding(true); movementSync.onGround.set(false);
        var follow = new FollowMovement(99, 8, false, movementSync.beginNavigationRequest());
        follow.init(); follow.onTick();
        entity.moveTo(new Vector3d(200, 0, 0));
        entity.getMetadata().put("shared_flags", (byte) 0);
        for (int i = 0; i < 60; i++) follow.onTick();
        assertEquals(new Vector3d(200, 0, 0), ((ElytraFlyMovement) child(follow)).getTarget());
        var field = FollowMovement.class.getDeclaredField("child");
        field.setAccessible(true); field.set(follow, null);
        movementSync.position.set(new Vector3d(200, 0, 0));
        movementSync.onGround.set(true); movementSync.setGliding(false);
        follow.onTick();
        var flying = FollowMovement.class.getDeclaredField("flying");
        flying.setAccessible(true);
        assertFalse(flying.getBoolean(follow));
        assertFalse(follow.isFinished());
    }

    @Test void cancellingFollowFlightInstallsIndependentSafetyLanding() throws Exception {
        installTarget(true);
        movementSync.getMovementController().pause();
        movementSync.setGliding(true);
        movementSync.onGround.set(false);
        movementSync.startFollowingNavigation(99, 8, false);
        var follow = assertInstanceOf(FollowMovement.class,
                movementSync.getMovementController().getMovements().getFirst());
        follow.init(); follow.onTick();
        assertInstanceOf(ElytraFlyMovement.class, child(follow));
        var controller = movementSync.getMovementController();
        controller.getMovements().remove(follow);
        var current = controller.getClass().getDeclaredField("currentMovement");
        current.setAccessible(true); current.set(controller, follow);
        movementSync.cancelNavigation();
        assertTrue(follow.isFinished());
        assertInstanceOf(ElytraFlyMovement.class,
                movementSync.getMovementController().getMovements().getFirst());
        assertNull(movementSync.getGotoRecovery().getTask());
    }

    @Test void inheritedGlideEntersFlightWithoutGroundPreparation() throws Exception {
        installTarget(true);
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        var follow = new FollowMovement(99, 8, false, movementSync.beginNavigationRequest());
        follow.init(); follow.onTick();
        assertInstanceOf(ElytraFlyMovement.class, child(follow));
        assertTrue(movementSync.isGliding());
    }
}
