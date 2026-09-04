package xin.bbtt;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;

final class NavigationRequestGenerationTest {
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
