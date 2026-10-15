package xin.bbtt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;
import org.joml.Vector3i;
import xin.bbtt.movement.Movement;

final class NavigationRequestGenerationTest {
    @Test
    void staleAbandonmentPreservesNewerGoalAndGeneration() {
        MovementSync movementSync = new MovementSync();
        long stale = movementSync.beginStaticNavigationRequest(new Vector3i(1, 2, 3));
        Vector3i newerGoal = new Vector3i(4, 5, 6);
        long current = movementSync.beginStaticNavigationRequest(newerGoal);

        movementSync.abandonNavigationRequest(stale);

        assertEquals(newerGoal, movementSync.getActiveGoal());
        assertEquals(current, movementSync.getNavigationGeneration());
    }

    @Test
    void currentAbandonmentClearsOnlyActiveGoal() {
        MovementSync movementSync = new MovementSync();
        long generation = movementSync.beginStaticNavigationRequest(new Vector3i(1, 2, 3));
        movementSync.setFollowTargetId(42);
        AtomicBoolean stopped = new AtomicBoolean();
        movementSync.getMovementController().pause();
        Movement queuedMovement = new Movement() {
            @Override public void init() {}
            @Override public void onTick() {}
            @Override public long getTime() { return -1; }
            @Override public void onStop() { stopped.set(true); }
        };
        movementSync.getMovementController().addMovement(queuedMovement);

        movementSync.abandonNavigationRequest(generation);

        assertNull(movementSync.getActiveGoal());
        assertEquals(generation, movementSync.getNavigationGeneration());
        assertEquals(42, movementSync.getFollowTargetId());
        assertFalse(stopped.get());
        assertTrue(movementSync.getMovementController().isPaused());
        assertEquals(java.util.List.of(queuedMovement), movementSync.getMovementController().getMovements());
    }

    @Test
    void staleGenerationCannotRunAuthorizedAction() {
        MovementSync movementSync = new MovementSync();
        long stale = movementSync.beginNavigationRequest();
        long current = movementSync.beginNavigationRequest();
        AtomicBoolean ran = new AtomicBoolean();

        assertFalse(movementSync.runIfNavigationRequestCurrent(stale, () -> ran.set(true)));
        assertFalse(ran.get());
        assertTrue(movementSync.runIfNavigationRequestCurrent(current, () -> ran.set(true)));
        assertTrue(ran.get());
    }

    @Test
    void requestSwitchWaitsForAnAuthorizedActionBoundary() throws Exception {
        MovementSync movementSync = new MovementSync();
        long generation = movementSync.beginNavigationRequest();
        CountDownLatch actionEntered = new CountDownLatch(1);
        CountDownLatch releaseAction = new CountDownLatch(1);
        CompletableFuture<Boolean> action = CompletableFuture.supplyAsync(() ->
            movementSync.runIfNavigationRequestCurrent(generation, () -> {
                actionEntered.countDown();
                try {
                    if (!releaseAction.await(2, TimeUnit.SECONDS)) {
                        throw new AssertionError("timed out waiting to release action");
                    }
                } catch (InterruptedException error) {
                    Thread.currentThread().interrupt();
                    throw new AssertionError(error);
                }
            })
        );
        assertTrue(actionEntered.await(2, TimeUnit.SECONDS));

        CompletableFuture<Long> replacement = CompletableFuture.supplyAsync(
            movementSync::beginNavigationRequest);
        Thread.sleep(50);
        assertFalse(replacement.isDone(),
            "request replacement must not interleave with an authorized action");

        releaseAction.countDown();
        assertTrue(action.get(2, TimeUnit.SECONDS));
        long newer = replacement.get(2, TimeUnit.SECONDS);
        assertTrue(newer > generation);
    }
}
