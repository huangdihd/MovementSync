package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.PlayerInput;

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
        WalkMovement movement = new WalkMovement(WalkMovement.Direction.FRONT, 1_000L);
        movement.init();
        assertEquals(new PlayerInput(true, false, false, false, false, false, false), movementSync.getInput());
        assertEquals(new Vector3d(), movementSync.velocity.get());

        // A collision may clear velocity while the key remains held.
        movementSync.velocity.set(new Vector3d());
        movement.onStop();

        assertEquals(PlayerInput.NONE, movementSync.getInput());
        assertEquals(new Vector3d(), movementSync.velocity.get());
    }

    @Test
    void stoppingHeldKeysDoesNotDamagePriorVelocity() {
        movementSync.velocity.set(new Vector3d(0.05, 0.0, 0.0));
        WalkMovement movement = new WalkMovement(WalkMovement.Direction.RIGHT, 1_000L);

        movement.init();
        assertEquals(0.05, movementSync.velocity.get().x, 1e-9);
        movement.onStop();

        assertEquals(0.05, movementSync.velocity.get().x, 1e-9);
        assertEquals(PlayerInput.NONE, movementSync.getInput());
    }

    @Test
    void concurrentVelocityChangeIsPreservedOnStop() {
        WalkMovement movement = new WalkMovement(WalkMovement.Direction.RIGHT, 1_000L);
        movement.init();
        movementSync.velocity.set(new Vector3d(-0.04, 0.0, 0.0));
        movement.onStop();

        assertEquals(-0.04, movementSync.velocity.get().x, 1e-9);
    }

    @Test
    void stopPreservesConcurrentJumpSneakAndUnownedKeys() {
        WalkMovement movement = new WalkMovement(WalkMovement.Direction.FRONT, 500L, true);
        movement.init();
        assertEquals(new PlayerInput(true, false, false, false, false, false, true), movementSync.getInput());
        movementSync.setInput(new PlayerInput(true, false, true, false, true, true, true));
        movement.onStop();

        assertEquals(new PlayerInput(false, false, true, false, true, true, false), movementSync.getInput());
    }

    @Test
    void alreadyHeldKeysRemainHeldAndRepeatedStopDoesNothing() {
        PlayerInput prior = new PlayerInput(true, false, false, false, true, true, true);
        movementSync.setInput(prior);
        WalkMovement movement = new WalkMovement(WalkMovement.Direction.FRONT, 500L, true);
        movement.init();
        movement.init();
        movement.onStop();
        assertEquals(prior, movementSync.getInput());
        movementSync.setInput(PlayerInput.NONE);
        movement.onStop();
        assertEquals(PlayerInput.NONE, movementSync.getInput());
    }

    @Test
    void pauseReleasesKeysAndResumeReacquiresThem() {
        WalkMovement movement = new WalkMovement(WalkMovement.Direction.BACK, 500L);
        movement.onStop();
        movement.init();
        PlayerInput held = movementSync.getInput();
        movement.onPause();
        assertEquals(PlayerInput.NONE, movementSync.getInput());
        movement.onResume();
        assertEquals(held, movementSync.getInput());
        assertEquals(500L, movement.getTime());
    }

    @Test
    void compassDirectionTurnsOnlyWhenMovementStarts() {
        movementSync.yaw.set(37f);
        WalkMovement movement = new WalkMovement(WalkMovement.Direction.WEST, 500L);
        assertEquals(37f, movementSync.yaw.get());
        movement.init();
        assertEquals(90f, movementSync.yaw.get());
        assertEquals(new PlayerInput(true, false, false, false, false, false, false), movementSync.getInput());
    }

    @Test
    void relativeDirectionDoesNotTurnOrRewriteVelocityOnTick() {
        movementSync.yaw.set(37f);
        WalkMovement movement = new WalkMovement(WalkMovement.Direction.LEFT, 500L);
        movement.init();
        movementSync.velocity.set(new Vector3d(0.12, -0.2, 0.3));
        movementSync.yaw.set(-45f);
        movement.onTick();
        assertEquals(-45f, movementSync.yaw.get());
        assertEquals(new Vector3d(0.12, -0.2, 0.3), movementSync.velocity.get());
        assertEquals(new PlayerInput(false, false, true, false, false, false, false), movementSync.getInput());
    }

    @Test
    void negativeDurationIsRejected() {
        assertThrows(IllegalArgumentException.class, () -> new WalkMovement(WalkMovement.Direction.FRONT, -1));
    }
}
