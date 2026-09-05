package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;

final class WalkMovementTest {
    private MovementSync movementSync;

    @BeforeEach
    void setUp() {
        movementSync = new MovementSync();
        movementSync.velocity.set(new Vector3d());
    }

    @AfterEach
    void tearDown() {
        MovementSync.INSTANCE = null;
    }

    @Test
    void collisionClearedAxisDoesNotBecomeReverseVelocityOnStop() {
        WalkMovement movement = new WalkMovement(new Vector3d(0.0, 0.0, -0.2159), 1_000L);
        movement.init();
        assertEquals(-0.2159, movementSync.velocity.get().z, 1e-9);

        // updateMotionTask clears a collided axis before the movement duration ends.
        movementSync.velocity.updateAndGet(current -> new Vector3d(current.x, current.y, 0.0));
        movement.onStop();

        assertEquals(0.0, movementSync.velocity.get().z, 1e-9);
    }

    @Test
    void unchangedAppliedVelocityIsRemovedWithoutDamagingPriorVelocity() {
        movementSync.velocity.set(new Vector3d(0.05, 0.0, 0.0));
        WalkMovement movement = new WalkMovement(new Vector3d(0.15, 0.0, 0.0), 1_000L);

        movement.init();
        assertEquals(0.20, movementSync.velocity.get().x, 1e-9);
        movement.onStop();

        assertEquals(0.05, movementSync.velocity.get().x, 1e-9);
    }

    @Test
    void concurrentVelocityChangeIsPreservedOnStop() {
        WalkMovement movement = new WalkMovement(new Vector3d(0.15, 0.0, 0.0), 1_000L);
        movement.init();

        movementSync.velocity.set(new Vector3d(-0.04, 0.0, 0.0));
        movement.onStop();

        assertEquals(-0.04, movementSync.velocity.get().x, 1e-9);
    }
}
