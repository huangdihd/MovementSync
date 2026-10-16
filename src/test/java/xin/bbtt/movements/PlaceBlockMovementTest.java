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
    void bridgeApproachesFaceThenWaitsForAimWithoutDrivingIntoLedge() {
        MovementSync sync = new MovementSync();
        for (int x = 0; x < 7; x++) {
            sync.position.set(new Vector3d(x + 0.5, 64, 0.5));
            sync.publishPlacementFrame(x + 0.5, 64, 0.5, -90, 0, 1.27);
            PlaceBlockMovement movement = new PlaceBlockMovement(
                    Vector3i.from(x + 1, 63, 0), Vector3i.from(x, 63, 0), Direction.EAST, true);
            movement.init();
            sync.publishPlacementFrame(x + 0.5, 64, 0.5, -90, 0, 1.27);
            movement.onTick();
            org.junit.jupiter.api.Assertions.assertTrue(sync.snapshotInput().forward());
            sync.position.set(new Vector3d(x + 1.05, 64, 0.5));
            sync.publishPlacementFrame(x + 1.05, 64, 0.5, -90, 0, 1.27);
            movement.onTick();
            assertEquals(new xin.bbtt.physics.PlayerInput(false, false, false, false, false, true, false),
                    sync.snapshotInput());
        }
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
