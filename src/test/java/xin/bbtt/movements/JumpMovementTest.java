package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;

final class JumpMovementTest {
    @AfterEach
    void tearDown() {
        MovementSync.INSTANCE = null;
    }

    @Test
    void initRequestsOneShotJumpWithoutDirectlyChangingVelocity() {
        class RecordingMovementSync extends MovementSync {
            private int jumpRequests;

            @Override
            public void jump() {
                jumpRequests++;
            }
        }
        RecordingMovementSync movementSync = new RecordingMovementSync();
        movementSync.onGround.set(false);
        movementSync.velocity.set(new Vector3d(0.1, -0.2, 0.3));
        JumpMovement movement = new JumpMovement();
        movement.init();
        movement.onTick();
        movement.onStop();

        assertEquals(1, movementSync.jumpRequests);
        assertEquals(new Vector3d(0.1, -0.2, 0.3), movementSync.velocity.get());
        assertEquals(0L, movement.getTime());
    }
}
