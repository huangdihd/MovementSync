package xin.bbtt.movements;

import java.util.Map;
import java.util.UUID;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Entity.Entity;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.movement.Movement;
import xin.bbtt.physics.FixedFlightTarget;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Owner regressions: loss after early touchdown is a new fixed search trip. */
final class FollowLostRollingTerrainTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");

    private FollowMovement start() throws Exception {
        movementSync.getMovementController().pause();
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                int floor = 64 + Math.floorMod((int) Math.floor(p.x / 8), 3);
                boolean tree = p.z >= 3 && p.z < 6 && Math.floorMod((int) Math.floor(p.x), 12) < 3
                        && p.y < floor + 5;
                return p.y < floor || tree ? STONE : AIR;
            }
        };
        assertTrue(xin.bbtt.pathfinding.StandablePositionResolver.feetY(world, 0, 64, 0).isPresent(), "launch support");
        assertTrue(xin.bbtt.pathfinding.StandablePositionResolver.feetY(world, 20, 66, 0).isPresent(), "destination support");
        set(movementSync, "world", world);
        movementSync.position.set(new Vector3d(.5, 64, .5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        movementSync.setGliding(false);
        Entity entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                20.5, 66, .5, 0, 0, 0, new Vector3d());
        entity.getMetadata().put("shared_flags", (byte) 0);
        world.getEntities().put(99, entity);
        FollowMovement follow = new FollowMovement(99, 2, false, movementSync.beginNavigationRequest());
        follow.init();
        return follow;
    }

    @Test void groundedLossDiscardsOldWalkingChildAndCooldownWithoutReportingArrival() throws Exception {
        var follow = start();
        Movement stale = new Movement() {
            @Override public void init() {}
            @Override public void onStop() {}
            @Override public void onTick() { fail("old entity-follow walking child must not tick after loss"); }
            @Override public long getTime() { return -1; }
        };
        try {
            set(follow, "child", stale);
            set(follow, "lowPlanRetryTicks", 20);
            set(follow, "walkingPlanAttempted", true);
            // No equipment: the fixed search must retain a walking alternative.
            inventory[6] = null;
            movementSync.inventoryManager.setContainerItems(0, inventory);
            movementSync.getWorld().getEntities().remove(99);
            follow.onTick();
            assertFalse(follow.isFinished(), "grounded far from last position is not arrival");
            assertNotSame(stale, get(follow, "child"));
            assertInstanceOf(PathMovement.class, get(follow, "child"));
            assertEquals(-1, get(get(follow, "child"), "requestedFollowTargetId"));
            assertEquals(-1, movementSync.getFollowTargetId());
            assertEquals(new org.joml.Vector3i(20, 66, 0), movementSync.getActiveGoal());
            assertEquals(0, get(follow, "lowPlanRetryTicks"));
            movementSync.position.set(new Vector3d(20.5, 66, .5));
            follow.onTick();
            assertTrue(follow.isFinished(), "only arrival on the ground reports loss");
        } finally { follow.onStop(); }
    }

    @Test void walkingSearchReacquisitionRestoresEntityRequestIdentity() throws Exception {
        var follow = start();
        var entity = movementSync.getWorld().getEntity(99);
        try {
            movementSync.setFollowTargetId(99);
            inventory[6] = null;
            movementSync.inventoryManager.setContainerItems(0, inventory);
            movementSync.getWorld().getEntities().remove(99);
            follow.onTick();
            assertInstanceOf(PathMovement.class, get(follow, "child"));
            assertEquals(-1, movementSync.getFollowTargetId());
            movementSync.getWorld().getEntities().put(99, entity);
            follow.onTick();
            assertEquals(99, movementSync.getFollowTargetId());
            assertInstanceOf(PathMovement.class, get(follow, "child"));
            assertEquals(99, get(get(follow, "child"), "requestedFollowTargetId"));
            assertFalse(follow.isFinished());
        } finally { follow.onStop(); }
    }

    @Test void airborneLossUsesFixedLandingDestinationAndCanReacquire() throws Exception {
        var follow = start();
        var entity = movementSync.getWorld().getEntity(99);
        try {
            movementSync.onGround.set(false);
            movementSync.position.set(new Vector3d(.5, 70, .5));
            movementSync.setGliding(true);
            movementSync.getWorld().getEntities().remove(99);
            follow.onTick();
            var child = assertInstanceOf(ElytraFlyMovement.class, get(follow, "child"));
            assertInstanceOf(FixedFlightTarget.class, get(child, "target"));
            assertEquals(entity.getPosition(), child.getTarget());
            assertFalse(follow.isFinished());
            movementSync.getWorld().getEntities().put(99, entity);
            follow.onTick();
            assertFalse(((xin.bbtt.physics.EntityFlightTarget) get(follow, "target")).lost());
            assertFalse((boolean) get(follow, "reportedLost"));
            assertFalse(follow.isFinished());
        } finally { follow.onStop(); }
    }

    @Test void earlySearchTouchdownFarFromDestinationDoesNotFinishLost() throws Exception {
        var follow = start();
        try {
            movementSync.onGround.set(false);
            movementSync.position.set(new Vector3d(.5, 70, .5));
            movementSync.setGliding(true);
            movementSync.getWorld().getEntities().remove(99);
            follow.onTick();
            ((Movement) get(follow, "child")).setFinished(true);
            movementSync.position.set(new Vector3d(.5, 64, .5));
            movementSync.velocity.set(new Vector3d());
            movementSync.onGround.set(true);
            movementSync.setGliding(false);
            follow.onTick();
            assertFalse(follow.isFinished(), "failed search touchdown is not arrival");
            assertFalse((boolean) get(follow, "flying"));
            follow.onTick();
            assertInstanceOf(PathMovement.class, get(follow, "child"));
            assertFalse(follow.isFinished());
        } finally { follow.onStop(); }
    }

    @Test void supersededGenerationDoesNotStartLostSearch() throws Exception {
        var follow = start();
        try {
            movementSync.getWorld().getEntities().remove(99);
            movementSync.beginNavigationRequest();
            follow.onTick();
            assertNull(get(follow, "child"));
            assertNull(get(follow, "lowPlan"));
            assertFalse((boolean) get(follow, "reportedLost"));
        } finally { follow.onStop(); }
    }

    @Test void groundedLossPhysicallyFliesRollingTerrainToLastDestination() throws Exception {
        var follow = start();
        follow.onStop();
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                int x = (int) Math.floor(p.x);
                int floor = 64 + (x > 12 && x < 88 ? Math.floorMod(x, 24) / 6 : 0);
                boolean tree = x > 24 && x < 80 && Math.floorMod(x, 24) < 3
                        && p.z >= 3 && p.z < 6 && p.y < floor + 5;
                return p.y < floor || tree ? STONE : AIR;
            }
        };
        set(movementSync, "world", world);
        Vector3d destination = new Vector3d(100.5, 64, .5);
        Entity entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                destination.x, destination.y, destination.z, 0, 0, 0, new Vector3d());
        entity.getMetadata().put("shared_flags", (byte) 0);
        world.getEntities().put(99, entity);
        inventory[36] = item("firework_rocket", 64);
        movementSync.inventoryManager.setContainerItems(0, inventory);
        follow = new FollowMovement(99, 2, false, movementSync.beginNavigationRequest());
        follow.init();
        world.getEntities().remove(99);
        var pose = xin.bbtt.physics.PlayerPose.STANDING;
        int deployments = 0, rockets = 0, rocketTicks = 0, flightTicks = 0, touchdowns = 0;
        double flightDistance = 0;
        boolean crossedHumps = false;
        try {
            for (int tick = 0; tick < 800 && !follow.isFinished(); tick++) {
                awaitPlan(follow);
                Object child = get(follow, "child");
                if (child instanceof ElytraFlyMovement) awaitPlan(child);
                follow.onTick();
                if (follow.isFinished()) {
                    assertTrue(movementSync.onGround.get());
                    assertTrue(movementSync.position.get().distance(destination) <= 2,
                            "search may finish only near the last destination");
                    break;
                }
                child = get(follow, "child");
                if (child instanceof ElytraFlyMovement flight) {
                    assertInstanceOf(FixedFlightTarget.class, get(flight, "target"));
                    assertEquals(destination, flight.getTarget());
                }
                var input = movementSync.snapshotInput();
                var packet = movementSync.pollBeforeMovementPacket();
                while (packet != null) {
                    if (packet instanceof org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket command
                            && command.getState() == org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerState.START_ELYTRA_FLYING) {
                        assertFalse(movementSync.onGround.get(), "deployment follows a physical jump");
                        assertTrue(movementSync.velocity.get().y <= 0);
                        deployments++;
                        movementSync.setGliding(true); // Simulated server acknowledgement only.
                    } else if (packet instanceof org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket) {
                        assertTrue(movementSync.isGliding());
                        assertEquals(0, rocketTicks);
                        inventory[36] = item("firework_rocket", 64 - ++rockets);
                        movementSync.inventoryManager.setContainerItems(0, inventory);
                        rocketTicks = 30;
                        Entity rocket = new Entity(9001, new UUID(0, 9001), EntityType.FIREWORK_ROCKET,
                                movementSync.position.get().x, movementSync.position.get().y,
                                movementSync.position.get().z, 0, 0, 0, new Vector3d());
                        rocket.getMetadata().put("attached_to_target", java.util.OptionalInt.of(movementSync.entityId));
                        world.getEntities().put(9001, rocket);
                    }
                    packet = movementSync.pollBeforeMovementPacket();
                }
                boolean gliding = movementSync.isGliding();
                var state = new xin.bbtt.physics.PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                        movementSync.onGround.get(), movementSync.fallDistance.get(), pose);
                var next = gliding
                        ? new xin.bbtt.physics.GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(),
                                movementSync.hasAttachedFirework(), input, movementSync.getMovementParameters())
                                .step(world, state, xin.bbtt.physics.PlayerPose.HALF_WIDTH, (double) (float) pose.height())
                        : new xin.bbtt.physics.WalkingPhysics(input, movementSync.yaw.get(), true, movementSync.getMovementParameters())
                                .step(world, state, xin.bbtt.physics.PlayerPose.HALF_WIDTH, (double) (float) pose.height());
                assertFalse(next.collidedX() || next.collidedZ(), "terrain impact at " + next.position());
                assertTrue(next.position().isFinite());
                if (gliding) {
                    flightTicks++;
                    flightDistance += Math.hypot(next.position().x - state.position().x, next.position().z - state.position().z);
                    if (next.position().x > 20 && next.position().x < 80) crossedHumps = true;
                }
                // Never teleport the bot or replace its velocity: all movement uses shared physics.
                movementSync.position.set(next.position());
                movementSync.velocity.set(next.velocity());
                movementSync.onGround.set(next.onGround());
                movementSync.fallDistance.set(next.fallDistance());
                pose = pose.afterTravel(world, next.position(), gliding, false, input.sneak());
                if (rocketTicks > 0 && --rocketTicks == 0) world.getEntities().remove(9001);
                if (gliding && next.onGround()) {
                    touchdowns++;
                    movementSync.setGliding(false);
                    world.getEntities().remove(9001);
                    rocketTicks = 0;
                }
            }
            String diagnostic = "position=" + movementSync.position.get() + " deployments=" + deployments
                    + " rockets=" + rockets + " flightTicks=" + flightTicks;
            assertTrue(follow.isFinished(), diagnostic);
            assertEquals(1, deployments, diagnostic);
            assertTrue(rockets > 0, diagnostic);
            assertTrue(flightTicks > 20 && flightDistance > 80 && crossedHumps, diagnostic);
            assertTrue(touchdowns > 0 && movementSync.onGround.get(), diagnostic);
            assertFalse(movementSync.isGliding(), diagnostic);
            assertTrue(movementSync.position.get().distance(destination) <= 2, diagnostic);
        } finally { follow.onStop(); world.getEntities().remove(9001); }
    }

    private static void awaitPlan(Object owner) throws Exception {
        Object pending = get(owner, "lowPlan");
        if (pending instanceof java.util.concurrent.CompletableFuture<?> future)
            future.get(30, java.util.concurrent.TimeUnit.SECONDS);
    }

    private static Object get(Object owner, String name) throws Exception {
        var field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(owner);
    }
    private static void set(Object owner, String name, Object value) throws Exception {
        var field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        field.set(owner, value);
    }
}
