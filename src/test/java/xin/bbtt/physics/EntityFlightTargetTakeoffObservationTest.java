package xin.bbtt.physics;

import java.util.UUID;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Entity.Entity;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class EntityFlightTargetTakeoffObservationTest {
    private final World world = new World() {
        @Override public boolean isOnGround(Vector3d p) { return true; }
    };

    private Entity entity(double x, Vector3d velocity) {
        var entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                x, 72, 25.5, 0, 0, 0, velocity);
        world.getEntities().put(99, entity);
        return entity;
    }

    @Test void sixteenBlocksPerSecondIsNotStationaryDuringFilterWarmup() {
        var entity = entity(-330, new Vector3d());
        var target = new EntityFlightTarget(99, 6, entity.getPosition());
        for (int tick = 0; tick < 40; tick++) {
            entity.moveTo(new Vector3d(entity.getPosition()).add(-.8, 0, 0));
            target.sample(world);
            assertEquals(.8, target.pursuitSpeed(), 1e-10);
            assertFalse(target.landingWithin(new Vector3d(-272.5, 72, 25.5), 6));
        }
        assertTrue(target.horizontalSpeed() > .79);
    }

    @Test void knownVelocitySurvivesNewFlightFirstObservation() {
        var entity = entity(-330, new Vector3d(-.8, 0, 0));
        var target = new EntityFlightTarget(99, 6, entity.getPosition(), entity.getMovement());
        target.sample(world);
        assertEquals(.64, target.horizontalSpeed(), 1e-10);
        assertTrue(target.pursuitSpeed() > .28);
        entity.getMovement().zero();
        assertEquals(.64, target.horizontalSpeed(), 1e-10, "seed must be copied");
    }

    @Test void reobservationDoesNotTreatUnseenTravelAsVelocity() {
        var entity = entity(-330, new Vector3d());
        var target = new EntityFlightTarget(99, 6, entity.getPosition());
        world.getEntities().remove(99);
        target.sample(world);
        entity.moveTo(new Vector3d(-500, 72, 25.5));
        world.getEntities().put(99, entity);
        target.sample(world);
        assertFalse(target.lost());
        assertEquals(0, target.horizontalSpeed());
        assertEquals(0, target.pursuitSpeed());
        assertEquals(entity.getPosition(), target.position());
    }
}
