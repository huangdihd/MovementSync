package xin.bbtt.movements;

import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Entity.Entity;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.EntityFlightTarget;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.world.World;

import java.util.Map;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.*;

final class LowAltitudeFollowMovementTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState(
            "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
            "minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");

    private static World cave() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            // Supply actual voxel geometry so collision clipping, ground support,
            // and support-exclusion queries all see the same floor and roof.
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return p.y < 64 || p.y >= 80 ? STONE : AIR;
            }
        };
    }

    private FollowMovement start() throws Exception {
        World cave = cave();
        var world = MovementSync.class.getDeclaredField("world");
        world.setAccessible(true);
        world.set(movementSync, cave);
        movementSync.position.set(new Vector3d(8, 64, 8));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        Entity entity = new Entity(99, UUID.randomUUID(), EntityType.ARMOR_STAND,
                8, 64, 28, 0, 0, 0, new Vector3d());
        entity.getMetadata().put("shared_flags", (byte) 0);
        cave.getEntities().put(99, entity);
        var follow = new FollowMovement(99, 8, false, movementSync.beginNavigationRequest());
        follow.init();
        entity.move(new Vector3d(0, 0, 3));
        follow.onTick();
        assertFalse(TravelMovement.isOpen(cave, movementSync.position.get()));
        return follow;
    }

    private static Object field(Object owner, String name) throws Exception {
        var field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(owner);
    }

    @SuppressWarnings("unchecked")
    private static CompletableFuture<LowAltitudeFlightPlan.Result> pending(FollowMovement follow) throws Exception {
        return (CompletableFuture<LowAltitudeFlightPlan.Result>) field(follow, "lowPlan");
    }

    @Test void caveFixtureProvidesConsistentCollisionAndExcludedSupport() {
        World cave = cave();
        var feet = new Vector3d(8.5, 64, 8.5);
        assertTrue(cave.isOnGround(feet));
        assertFalse(cave.isBoxColliding(8.2, 64, 8.2, 8.8, 65.8, 8.8));
        assertTrue(cave.isBoxColliding(8.2, 63.9, 8.2, 8.8, 65.8, 8.8));
        assertTrue(cave.isBoxColliding(8.2, 79, 8.2, 8.8, 80.1, 8.8));
        assertEquals(64, cave.findHighestCollisionTop(8.2, 63.9, 8.2, 8.8, 64.1, 8.8)
                .orElseThrow());
        assertTrue(cave.findHighestCollisionTopExcluding(8.2, 63.9, 8.2, 8.8, 64.1, 8.8,
                Set.of(new Vector3i(8, 63, 8))).isEmpty());
    }

    @Test void groundPursuitUnderRoofPreparesPredictedOffsetRouteAndStartsFlight() throws Exception {
        var follow = start();
        try {
            var plan = pending(follow);
            assertNotNull(plan, "roof must schedule worker planning rather than permanently gate takeoff");
            var result = plan.get(10, TimeUnit.SECONDS);
            assertNotNull(result);
            var target = (EntityFlightTarget) field(follow, "target");
            var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(), true);
            assertEquals(target.aim(state), result.destination());
            assertEquals(64, result.destination().y, "do not replace ground target with cruise height");
            follow.onTick();
            var flight = assertInstanceOf(ElytraFlyMovement.class, field(follow, "child"));
            assertTrue((boolean) field(flight, "lowAltitude"));
        } finally { follow.onStop(); }
    }

    @Test void stoppedFollowDiscardsPendingRouteAndCannotPublishFlight() throws Exception {
        var follow = start();
        follow.onStop();
        assertNull(pending(follow));
        follow.onTick();
        assertFalse(field(follow, "child") instanceof ElytraFlyMovement);
    }

    @Test void supersededGenerationCannotPublishCompletedRoute() throws Exception {
        var follow = start();
        try {
            var plan = pending(follow);
            assertNotNull(plan);
            plan.get(10, TimeUnit.SECONDS);
            movementSync.beginNavigationRequest();
            follow.onTick();
            assertNull(pending(follow));
            assertFalse(field(follow, "child") instanceof ElytraFlyMovement);
        } finally { follow.onStop(); }
    }
}
