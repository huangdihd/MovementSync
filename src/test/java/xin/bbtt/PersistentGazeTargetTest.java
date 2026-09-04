package xin.bbtt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import java.lang.reflect.Field;
import java.util.UUID;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import xin.bbtt.Entity.Entity;
import xin.bbtt.world.World;

final class PersistentGazeTargetTest {
    @AfterEach
    void resetSingleton() {
        MovementSync.INSTANCE = null;
    }

    @Test
    void blockTargetAppliesOnlyWhenOrientationIsIdleAndThenResumes() {
        MovementSync movementSync = new MovementSync();
        movementSync.position.set(new Vector3d(0.5, 64.0, 0.5));
        movementSync.yaw.set(12.0f);
        movementSync.pitch.set(3.0f);
        movementSync.setBlockGazeTarget(new Vector3i(3, 65, 0));

        movementSync.applyPersistentGaze(true);
        assertEquals(12.0f, movementSync.yaw.get(), 0.001f,
            "an orientation-sensitive action must temporarily override persistent gaze");

        movementSync.applyPersistentGaze(false);
        assertEquals(-90.0f, movementSync.yaw.get(), 0.001f,
            "persistent gaze must resume as soon as the competing action ends");
        assertTrue(movementSync.describeGazeTarget().contains("block=(3,65,0)"));
    }

    @Test
    void clearRemovesPersistentTarget() {
        MovementSync movementSync = new MovementSync();
        movementSync.setBlockGazeTarget(new Vector3i(1, 2, 3));

        movementSync.clearGazeTarget();

        assertEquals("none", movementSync.describeGazeTarget());
    }

    @Test
    void entityTargetTracksItsLatestPosition() throws Exception {
        MovementSync movementSync = new MovementSync();
        movementSync.position.set(new Vector3d(0.5, 64.0, 0.5));
        World world = new World();
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        Entity entity = new Entity(
            7, UUID.randomUUID(), EntityType.PLAYER,
            3.5, 64.0, 0.5, 0.0f, 0.0f, 0.0f, new Vector3d());
        world.getEntities().put(7, entity);
        movementSync.setEntityGazeTarget(7);

        movementSync.applyPersistentGaze(false);
        assertEquals(-90.0f, movementSync.yaw.get(), 0.001f);

        entity.moveTo(new Vector3d(0.5, 64.0, 3.5));
        movementSync.applyPersistentGaze(false);
        assertEquals(0.0f, movementSync.yaw.get(), 0.001f);
        assertTrue(movementSync.describeGazeTarget().contains("entity_id=7"));
    }
}
