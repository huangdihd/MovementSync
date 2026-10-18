package xin.bbtt.movements;

import java.util.Map;
import java.util.UUID;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Entity.Entity;
import xin.bbtt.ElytraCommandTestSupport;
import xin.bbtt.physics.EntityFlightTarget;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class FollowAirborneObservationRegressionTest extends ElytraCommandTestSupport {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState("minecraft:air", 0, Map.of(), "empty", 0, false, "AIR");

    @Test void airborneObservationTerminatesOnSupportBelowRatherThanRedeploying() throws Exception {
        movementSync.getMovementController().pause();
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) { return p.y < 64 ? STONE : AIR; }
        };
        set(movementSync, "world", world);
        movementSync.position.set(new Vector3d(.5, 70, .5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(false);
        movementSync.setGliding(true);
        var entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                20.5, 85, .5, 0, 0, 0, new Vector3d());
        world.getEntities().put(99, entity);
        var follow = new FollowMovement(99, 2, false, movementSync.beginNavigationRequest());
        follow.init();
        try {
            world.getEntities().remove(99);
            follow.onTick();
            var child = assertInstanceOf(ElytraFlyMovement.class, get(follow, "child"));
            assertEquals(new Vector3d(20.5, 64, .5), child.getTarget());
            assertEquals(new Vector3d(20.5, 85, .5), ((EntityFlightTarget) get(follow, "target")).position());
            assertFalse(follow.isFinished());
            movementSync.position.set(new Vector3d(20.5, 64, .5));
            movementSync.onGround.set(true);
            movementSync.setGliding(false);
            follow.onTick();
            assertTrue(follow.isFinished());
            for (int i = 0; i < 40; i++) follow.onTick();
            assertNull(get(follow, "child"), "completed last-position search cannot launch again");
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
