package xin.bbtt.movements;

import java.util.UUID;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Entity.Entity;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.movement.Movement;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class FollowTakeoffMovingTargetTest extends ElytraCommandTestSupport {
    @Test void movingTargetNeverCommitsLandingDuringFirstFlightSamples() throws Exception {
        movementSync.getMovementController().pause();
        World world = new World() {
            @Override public boolean isOnGround(Vector3d p) { return true; }
        };
        set(movementSync, "world", world);
        movementSync.position.set(new Vector3d(-272.5, 75, 25.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        var entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                -330, 72, 25.5, 0, 0, 0, new Vector3d());
        world.getEntities().put(99, entity);
        var follow = new FollowMovement(99, 6, false, movementSync.beginNavigationRequest());
        follow.init();
        // Isolate owner decisions from steering: this represents the live child
        // immediately after takeoff, before the target EMA has warmed up.
        Movement pursuit = new Movement() {
            @Override public void init() { }
            @Override public void onTick() { }
            @Override public long getTime() { return -1; }
            @Override public void onStop() { }
        };
        set(follow, "child", pursuit);
        set(follow, "flying", true);
        try {
            for (int tick = 0; tick < 40; tick++) {
                entity.moveTo(new Vector3d(entity.getPosition()).add(-.8, 0, 0));
                follow.onTick();
                assertFalse((boolean) get(follow, "landing"), "no commit/abort cycle at tick " + tick);
                assertSame(pursuit, get(follow, "child"));
            }
        } finally { follow.onStop(); }
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
