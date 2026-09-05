package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.cloudburstmc.math.vector.Vector3i;
import org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;

final class PlaceBlockMovementTest {
    @AfterEach
    void resetSingleton() {
        MovementSync.INSTANCE = null;
    }

    @Test
    void initializationFacesClickedSurfaceWithoutQueueingALaterLookAction() {
        MovementSync movementSync = new MovementSync();
        movementSync.position.set(new Vector3d(0.5, 64.0, 0.5));
        PlaceBlockMovement movement = new PlaceBlockMovement(
            Vector3i.from(2, 64, 0),
            Vector3i.from(1, 64, 0),
            Direction.EAST,
            false
        );

        movement.init();

        assertEquals(-90.0f, movementSync.yaw.get(), 0.001f);
        assertNull(movementSync.getMovementController().getCurrentMovement(),
            "facing must happen before placement rather than via a queued LookAtMovement");
    }
}
