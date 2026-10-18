package xin.bbtt.physics;

import java.util.UUID;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Entity.Entity;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class EntityFlightTargetLandingPreviewTest {
    @Test void liveGlidingUpdatesRemainTrackedBeyondSevenSecondsAndSilenceIsNotRemoval() {
        World world = new World();
        var entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                0, 85, 0, 0, 0, 0, new Vector3d());
        entity.getMetadata().put("shared_flags", (byte) 0x80);
        world.getEntities().put(99, entity);
        var target = new EntityFlightTarget(99, 2, entity.getPosition());
        for (int tick = 0; tick < 400; tick++) {
            // Both relative-move and position-sync packet handlers update this entity position.
            entity.moveTo(new Vector3d(tick, 85, 0));
            target.sample(world);
            assertFalse(target.lost());
            assertTrue(target.gliding());
            assertEquals(entity.getPosition(), target.position());
        }
        for (int tick = 0; tick < 400; tick++) target.sample(world);
        assertFalse(target.lost(), "stationary packet silence is not a server removal");
        world.getEntities().remove(99);
        target.sample(world);
        assertTrue(target.lost());
    }

    @Test void stoppingPreviewSettlesAndLandingFootprintAbsorbsRidgeJitter() {
        World world = new World() {
            @Override public boolean isOnGround(Vector3d p) { return true; }
        };
        var entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                0, 64, 0, 0, 0, 0, new Vector3d());
        world.getEntities().put(99, entity);
        var target = new EntityFlightTarget(99, 2, entity.getPosition());
        for (int tick = 1; tick <= 40; tick++) {
            entity.moveTo(new Vector3d(tick, 64, 0));
            target.sample(world);
        }
        assertFalse(target.landingWithin(target.position(), 2));
        var anchor = target.position();
        for (int tick = 0; tick < 20; tick++) target.sample(world);
        assertTrue(target.landed());
        assertTrue(target.landingWithin(anchor, 2));
        for (int tick = 0; tick < 80; tick++) {
            entity.moveTo(new Vector3d(anchor).add(tick % 2 == 0 ? .1 : -.1, tick % 2, 0));
            target.sample(world);
            assertTrue(target.landingWithin(anchor, 2 + PlayerPose.HALF_WIDTH));
        }
        entity.moveTo(new Vector3d(anchor).add(4, 0, 0));
        target.sample(world);
        assertFalse(target.landingWithin(anchor, 2 + PlayerPose.HALF_WIDTH));
    }
}
