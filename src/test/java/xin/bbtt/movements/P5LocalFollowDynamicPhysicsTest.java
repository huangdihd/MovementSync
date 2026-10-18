package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.Map;
import java.util.OptionalInt;
import java.util.UUID;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Entity.Entity;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.flight.LocalGotoController;
import xin.bbtt.flight.ScopeRouter;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.physics.EntityFlightTarget;
import xin.bbtt.physics.GlidingPhysics;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/** P5 dynamic observations: real follow owner, asynchronous provider and completed physics clock. */
final class P5LocalFollowDynamicPhysicsTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState(
            "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
            "minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");
    private static final int TARGET = 99;
    private static final int ROCKET = 9001;

    private static class Terrain extends World {
        private final boolean wall;
        Terrain(boolean wall) { this.wall = wall; }
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d p) {
            // The turn is before a solid wall, not through an entity-only obstacle.
            return p.y < 64 || p.y >= 90 || wall && p.x >= 52 && p.x < 55
                    ? STONE : AIR;
        }
    }

    @Test void movingTargetTurnsAlongWallWithoutImpactOrLosingPursuit() throws Exception {
        try (Flight flight = new Flight(new Terrain(true))) {
            for (int tick = 0; tick < 220; tick++) {
                // Fifty ticks along +X, then a right-angle turn along +Z eight blocks before the wall.
                flight.target.moveTo(tick < 50
                        ? new Vector3d(14 + .6 * (tick + 1), 74, .5)
                        : new Vector3d(44, 74, .5 + .6 * (tick - 49)));
                flight.tick(tick);
                assertTrue(movementSync.position.get().x + PlayerPose.HALF_WIDTH < 52,
                        "must turn on the near side of the wall: " + flight.context(tick));
                if (tick >= 30) assertTrue(horizontal(movementSync.position.get(), flight.target.getPosition()) < 40,
                        "bounded pursuit through the corner: " + flight.context(tick));
            }
            assertTrue(movementSync.position.get().z > 80, "must physically follow the new heading");
            assertTrue(flight.distance > 100, "must traverse both legs, not hover safely at the corner");
            flight.assertLocalTracking();
        }
    }

    @Test void targetTeleportReplacesOldHeadingAndReacquiresMovingTarget() throws Exception {
        try (Flight flight = new Flight(new Terrain(false))) {
            for (int tick = 0; tick < 240; tick++) {
                // Only the target teleports. The bot remains entirely physics-driven.
                flight.target.moveTo(tick < 60
                        ? new Vector3d(14 + .6 * (tick + 1), 74, .5)
                        : new Vector3d(50, 74, 44.5 + .6 * (tick - 60)));
                flight.tick(tick);
                double lag = horizontal(movementSync.position.get(), flight.target.getPosition());
                assertTrue(lag < 75, "teleport must not cause runaway pursuit: " + flight.context(tick));
                if (tick >= 150) assertTrue(lag < 35,
                        "must reacquire rather than pursue the pre-teleport extrapolation: " + flight.context(tick));
            }
            assertTrue(movementSync.position.get().z > 125, "must reach the post-teleport moving corridor");
            assertTrue(flight.distance > 130, "reacquisition must be physical progress");
            flight.assertLocalTracking();
        }
    }

    @Test void oldWorldWorkerCannotPublishAfterReplacementAndNewWorldStillTracks() throws Exception {
        Thread testThread = Thread.currentThread();
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        Terrain oldWorld = new Terrain(false) {
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (Thread.currentThread() != testThread) {
                    entered.countDown();
                    try {
                        if (!release.await(30, TimeUnit.SECONDS)) throw new AssertionError("worker release timed out");
                    } catch (InterruptedException interrupted) {
                        Thread.currentThread().interrupt();
                        throw new AssertionError(interrupted);
                    }
                }
                return super.getBlockStateAt(p);
            }
        };
        try (Flight flight = new Flight(oldWorld)) {
            try {
                flight.follow.onTick();
                LocalGotoController provider = flight.provider();
                assertTrue(entered.await(10, TimeUnit.SECONDS), "old-world planning must actually be in flight");
                assertNull(provider.latest(), "worker is held before publication");
                Terrain replacement = new Terrain(true);
                replacement.getEntities().put(TARGET, flight.target);
                set(movementSync, "world", replacement);
                flight.world = replacement;
                release.countDown();
                assertTrue(provider.awaitIdle(30, TimeUnit.SECONDS));
                assertNull(provider.latest(), "old World result must be discarded even at the same generation and tick");
                // Complete real physics before requesting another sample; never advance a synthetic clock alone.
                flight.step(0);
                for (int tick = 1; tick <= 160; tick++) {
                    flight.target.moveTo(new Vector3d(14, 74, .5 + tick * .6));
                    flight.tick(tick);
                    assertTrue(movementSync.position.get().x + PlayerPose.HALF_WIDTH < 52,
                            "replacement-world wall must remain authoritative: " + flight.context(tick));
                    if (tick > 50) assertTrue(horizontal(movementSync.position.get(), flight.target.getPosition()) < 35,
                            "fresh World must resume tracking: " + flight.context(tick));
                }
                assertTrue(movementSync.position.get().z > 70, "fresh worker must make physical progress");
                flight.assertLocalTracking();
            } finally {
                release.countDown();
            }
        }
    }

    private final class Flight implements AutoCloseable {
        private World world;
        private final Entity target;
        private final FollowMovement follow;
        private int rocketTicks;
        private int rockets;
        private int safeSamples;
        private double distance;

        Flight(World world) throws Exception {
            movementSync.getMovementController().pause();
            this.world = world;
            set(movementSync, "world", world);
            movementSync.position.set(new Vector3d(8, 74, .5));
            movementSync.velocity.set(new Vector3d(.6, 0, 0));
            movementSync.onGround.set(false);
            movementSync.yaw.set(-90f);
            movementSync.setGliding(true);
            inventory[36] = item("firework_rocket", 64);
            movementSync.inventoryManager.setContainerItems(0, inventory);
            target = new Entity(TARGET, new UUID(0, TARGET), EntityType.ARMOR_STAND,
                    14, 74, .5, 0, 0, 0, new Vector3d());
            target.getMetadata().put("shared_flags", (byte) 0x80);
            world.getEntities().put(TARGET, target);
            // Deliberately do not enable the feature here: these tests exercise its shipped default.
            assertEquals(ScopeRouter.Route.LOCAL,
                    movementSync.getFlightScopeRouter().route(ScopeRouter.Scope.LOCAL_FOLLOW, false, false));
            follow = new FollowMovement(TARGET, 6, false, movementSync.beginNavigationRequest());
            follow.init();
        }

        LocalGotoController provider() throws Exception {
            Object child = field(follow, "child");
            assertInstanceOf(ElytraFlyMovement.class, child, "live gliding target must own flight");
            return (LocalGotoController) field(child, "localGoto");
        }

        void tick(int tick) throws Exception {
            follow.onTick();
            LocalGotoController provider = provider();
            assertTrue(provider.awaitIdle(30, TimeUnit.SECONDS), "local worker did not finish");
            var result = provider.latest();
            assertNotNull(result, "local provider must really run, not just pass a routing flag assertion");
            if (result.status() == LocalGotoController.Status.SAFE_PREFIX) safeSamples++;
            assertNotEquals(LocalGotoController.Status.LANDED, result.status(), "live target cannot request touchdown");
            assertFalse(follow.isFinished(), context(tick));
            assertFalse((boolean) field(follow, "landing"), context(tick));
            assertFalse(((EntityFlightTarget) field(follow, "target")).lost(), context(tick));
            step(tick);
        }

        void step(int tick) {
            var input = movementSync.snapshotInput();
            var packet = movementSync.pollBeforeMovementPacket();
            while (packet != null) {
                Bot.INSTANCE.getSession().send(packet);
                assertFalse(packet instanceof ServerboundPlayerCommandPacket,
                        "acknowledged flight must not redeploy: " + context(tick));
                if (packet instanceof ServerboundUseItemPacket) {
                    assertEquals(0, rocketTicks, "attached rockets must not stack");
                    assertTrue(++rockets < 64, "bounded test inventory");
                    inventory[36] = item("firework_rocket", 64 - rockets);
                    movementSync.inventoryManager.setContainerItems(0, inventory);
                    Entity rocket = new Entity(ROCKET, new UUID(0, ROCKET), EntityType.FIREWORK_ROCKET,
                            movementSync.position.get().x, movementSync.position.get().y,
                            movementSync.position.get().z, 0, 0, 0, new Vector3d());
                    rocket.getMetadata().put("attached_to_target", OptionalInt.of(movementSync.entityId));
                    world.getEntities().put(ROCKET, rocket);
                    rocketTicks = 30;
                }
                packet = movementSync.pollBeforeMovementPacket();
            }
            assertTrue(movementSync.isGliding(), context(tick));
            var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                    movementSync.onGround.get(), movementSync.fallDistance.get(), PlayerPose.GLIDING);
            var next = new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(),
                    movementSync.hasAttachedFirework(), input, movementSync.getMovementParameters())
                    .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            assertFalse(next.collidedX() || next.collidedZ(), "wall impact: " + context(tick));
            assertFalse(next.onGround(), "moving gliding target must not cause touchdown: " + context(tick));
            assertTrue(next.position().y >= 64 && next.position().y + PlayerPose.GLIDING.height() <= 90 + 1.e-6,
                    "floor/roof penetration: " + context(tick));
            distance += next.position().distance(state.position());
            movementSync.position.set(next.position());
            movementSync.velocity.set(next.velocity());
            movementSync.onGround.set(next.onGround());
            movementSync.fallDistance.set(next.fallDistance());
            movementSync.completePhysicsStep();
            if (rocketTicks > 0 && --rocketTicks == 0) world.getEntities().remove(ROCKET);
        }

        String context(int tick) {
            return "tick=" + tick + " bot=" + movementSync.position.get() + " target=" + target.getPosition();
        }

        void assertLocalTracking() {
            assertTrue(safeSamples > 20, "repeated local safe prefixes must actually drive pursuit: " + safeSamples);
            assertTrue(rockets > 0, "exercise admitted rocket actions and real attachment physics");
        }

        @Override public void close() {
            follow.onStop();
            world.getEntities().remove(ROCKET);
        }
    }

    private static double horizontal(Vector3d a, Vector3d b) {
        return Math.hypot(a.x - b.x, a.z - b.z);
    }

    private static Object field(Object owner, String name) throws Exception {
        Field field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(owner);
    }

    private static void set(Object owner, String name, Object value) throws Exception {
        Field field = owner.getClass().getDeclaredField(name);
        field.setAccessible(true);
        field.set(owner, value);
    }
}
