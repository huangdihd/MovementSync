package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import xin.bbtt.Entity.Entity;
import xin.bbtt.world.World;
import java.util.UUID;
import static org.junit.jupiter.api.Assertions.*;

class EntityFlightTargetTest {
    @Test void predictionDistanceSmoothingAndLoss() {
        World world = new World() {
            @Override public boolean isOnGround(Vector3d p) { return p.y == 0; }
        };
        Entity entity = new Entity(99, UUID.randomUUID(), EntityType.ARMOR_STAND,
                0, 0, 0, 0, 0, 0, new Vector3d());
        world.getEntities().put(99, entity);
        var target = new EntityFlightTarget(99, 3, new Vector3d());
        entity.moveTo(new Vector3d(1, 0, 0));
        target.sample(world);
        assertEquals(0.2, target.horizontalSpeed());
        assertTrue(target.landed());
        var observer = new PhysicsState(new Vector3d(-9, 0, 0), new Vector3d(0.8, 0, 0), false);
        assertEquals(new Vector3d(0, 0, 0), target.aim(observer));
        entity.getMetadata().put("shared_flags", (byte)0x80);
        target.sample(world);
        assertTrue(target.gliding());
        assertFalse(target.landed());
        world.getEntities().remove(99);
        target.sample(world);
        assertTrue(target.lost());
        assertEquals(new Vector3d(1, 0, 0), target.aim(observer));
        assertEquals(new Vector3d(), target.velocity());
    }

    @Test void returningEntityClearsLossWithoutChangingItsIdentity() {
        World world = new World();
        var target = new EntityFlightTarget(99, 6, new Vector3d(-60, 64, 28));
        target.sample(world);
        assertTrue(target.lost());
        Entity entity = new Entity(99, UUID.randomUUID(), EntityType.ARMOR_STAND,
                -80, 65, 28, 0, 0, 0, new Vector3d());
        world.getEntities().put(99, entity);
        target.sample(world);
        assertFalse(target.lost());
        assertEquals(entity.getPosition(), target.position());
    }

    @Test void fixedTargetDefensivelyCopiesAndDoesNotPredict() {
        Vector3d original = new Vector3d(1, 2, 3);
        var target = new FixedFlightTarget(original);
        original.zero();
        target.position().zero();
        assertEquals(new Vector3d(1, 2, 3), target.aim(new PhysicsState(new Vector3d(), new Vector3d(8), false)));
        assertEquals(new Vector3d(), target.velocity());
        assertTrue(target.landed());
        assertFalse(target.lost());
    }
}
