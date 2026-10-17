package xin.bbtt.movements;

import java.util.Map;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Entity.Entity;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Stationary follow must land inside the tunnel, not seek its roof. */
final class FollowCorridorLandingRegressionTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");

    private FollowMovement start() throws Exception {
        movementSync.getMovementController().pause();
        World corridor = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return p.x < 0 || p.x >= 9 || p.y < 61 || p.y >= 76 ? STONE : AIR;
            }
        };
        set(movementSync, "world", corridor);
        movementSync.position.set(new Vector3d(4.5, 64, 8.5));
        movementSync.velocity.set(new Vector3d(0, -.05, .25));
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        Entity entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                4.5, 61, 18.5, 0, 0, 0, new Vector3d());
        entity.getMetadata().put("shared_flags", (byte) 0);
        corridor.getEntities().put(99, entity);
        FollowMovement follow = new FollowMovement(99, 2, false, movementSync.beginNavigationRequest());
        movementSync.setFollowTargetId(99);
        follow.init();
        return follow;
    }

    private static Object get(Object owner, String name) throws Exception {
        var f = owner.getClass().getDeclaredField(name); f.setAccessible(true); return f.get(owner);
    }
    private static void set(Object owner, String name, Object value) throws Exception {
        var f = owner.getClass().getDeclaredField(name); f.setAccessible(true); f.set(owner, value);
    }
    private static void awaitPlan(Object owner) throws Exception {
        Object plan = get(owner, "lowPlan");
        if (plan instanceof CompletableFuture<?> pending) pending.get(20, TimeUnit.SECONDS);
    }

    @Test void stoppedTargetLandsWithinTenSecondsUsingActualCorridorPhysics() throws Exception {
        var follow = start();
        try {
            for (int tick = 0; tick < 200 && !movementSync.onGround.get(); tick++) {
                follow.onTick();
                var flight = assertInstanceOf(ElytraFlyMovement.class, get(follow, "child"));
                assertTrue((boolean) get(flight, "lowAltitude"));
                assertEquals(61, flight.getTarget().y, "land on tunnel floor, never roof");
                awaitPlan(flight);
                var state = new PhysicsState(movementSync.position.get(), movementSync.velocity.get(), false,
                        movementSync.fallDistance.get(), PlayerPose.GLIDING);
                var next = new GlidingPhysics(movementSync.yaw.get(), movementSync.pitch.get(), false,
                        PlayerInput.NONE, movementSync.getMovementParameters())
                        .step(movementSync.getWorld(), state, (double) .6f / 2, (double) .6f);
                assertFalse(next.collidedX() || next.collidedZ(), "wall collision at " + next.position());
                movementSync.position.set(next.position());
                movementSync.velocity.set(next.velocity());
                movementSync.onGround.set(next.onGround());
                movementSync.fallDistance.set(next.fallDistance());
            }
            assertTrue(movementSync.onGround.get(), "stationary target must not hover for ten seconds");
            movementSync.setGliding(false);
            follow.onTick();
            assertFalse((boolean) get(follow, "flying"));
            assertFalse(follow.isFinished(), "touchdown must preserve follow intent");
        } finally { follow.onStop(); }
    }

    @Test void failedAirborneChildImmediatelyGetsLandingOwner() throws Exception {
        var follow = start();
        try {
            follow.onTick();
            ((Movement) get(follow, "child")).setFinished(true);
            follow.onTick();
            assertTrue((boolean) get(follow, "flying"));
            var child = assertInstanceOf(ElytraFlyMovement.class, get(follow, "child"));
            assertTrue((boolean) get(child, "safeLandingLeg"));
            assertFalse(follow.isFinished());
        } finally { follow.onStop(); }
    }

    @Test void earlyTouchdownRestartsWalkingEvenAfterPreviousWalkingAttempt() throws Exception {
        var follow = start();
        try {
            follow.onTick();
            set(follow, "walkingPlanAttempted", true);
            movementSync.position.set(new Vector3d(4.5, 61, 8.5));
            movementSync.velocity.set(new Vector3d());
            movementSync.onGround.set(true);
            movementSync.setGliding(false);
            follow.onTick();
            assertFalse((boolean) get(follow, "flying"));
            follow.onTick();
            assertInstanceOf(PathMovement.class, get(follow, "child"));
            assertFalse(follow.isFinished());
        } finally { follow.onStop(); }
    }
}
