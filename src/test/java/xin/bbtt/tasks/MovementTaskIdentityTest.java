package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.movement.MovementController;

final class MovementTaskIdentityTest {
    @AfterEach
    void resetSingleton() {
        if (MovementSync.INSTANCE != null && MovementSync.INSTANCE.movementService != null) {
            MovementSync.INSTANCE.movementService.shutdownNow();
        }
        MovementSync.INSTANCE = null;
    }

    @Test
    void staleActivationCannotStopTheSameMovementObjectAfterItIsRequeued() throws Exception {
        MovementSync movementSync = new MovementSync();
        movementSync.movementService = Executors.newScheduledThreadPool(3);
        ObservingMovementController controller = new ObservingMovementController();
        BlockingFirstActivationMovement repeated = new BlockingFirstActivationMovement();
        controller.watch(repeated);
        ImmediateMovement replacement = new ImmediateMovement();

        try {
            controller.addMovement(repeated);
            assertTrue(repeated.firstActivationEntered.await(1, TimeUnit.SECONDS));

            controller.insertMovement(replacement);
            assertTrue(repeated.secondActivationEntered.await(1, TimeUnit.SECONDS),
                "the same movement object should resume as a new scheduled activation");

            repeated.releaseFirstActivation.countDown();
            assertTrue(repeated.firstActivationLeaving.await(1, TimeUnit.SECONDS));
            assertTrue(controller.watchedFinishAttempt.await(1, TimeUnit.SECONDS),
                "the stale task must reach its guarded finish attempt before assertions");

            assertSame(repeated, controller.getCurrentMovement(),
                "the stale task must not stop a later activation of the same movement object");
            assertEquals(1, repeated.stopCalls.get(),
                "only insertMovement should have stopped the first activation");
        } finally {
            repeated.releaseFirstActivation.countDown();
            controller.cancelAll();
        }
    }

    @Test
    void failingStaleTaskDoesNotStopDistinctReplacementMovement() throws Exception {
        MovementController controller = new MovementController();
        RecordingMovement stale = new RecordingMovement();
        stale.throwOnTick = true;
        RecordingMovement replacement = new RecordingMovement();
        setCurrentMovement(controller, replacement);

        new MovementTask(stale, controller, -1L).run();

        assertSame(replacement, controller.getCurrentMovement(),
            "a stale task's exception must not finish the replacement movement");
        assertEquals(0, replacement.stopCalls,
            "the replacement movement must not receive onStop from a stale task's exception");
    }

    private static void setCurrentMovement(MovementController controller, Movement movement)
            throws ReflectiveOperationException {
        Field lockField = MovementController.class.getDeclaredField("stateLock");
        lockField.setAccessible(true);
        Object stateLock = lockField.get(controller);
        Field movementField = MovementController.class.getDeclaredField("currentMovement");
        movementField.setAccessible(true);
        synchronized (stateLock) {
            movementField.set(controller, movement);
        }
    }

    private static final class BlockingFirstActivationMovement extends Movement {
        private final CountDownLatch firstActivationEntered = new CountDownLatch(1);
        private final CountDownLatch releaseFirstActivation = new CountDownLatch(1);
        private final CountDownLatch firstActivationLeaving = new CountDownLatch(1);
        private final CountDownLatch secondActivationEntered = new CountDownLatch(1);
        private final AtomicInteger tickCalls = new AtomicInteger();
        private final AtomicInteger stopCalls = new AtomicInteger();

        @Override
        public void init() {}

        @Override
        public void onTick() {
            if (tickCalls.incrementAndGet() != 1) {
                secondActivationEntered.countDown();
                return;
            }
            firstActivationEntered.countDown();
            boolean released = false;
            while (!released) {
                try {
                    released = releaseFirstActivation.await(1, TimeUnit.SECONDS);
                } catch (InterruptedException ignored) {
                    // Deliberately model code that does not exit immediately when
                    // ScheduledFuture.cancel(true) interrupts the old activation.
                }
            }
            firstActivationLeaving.countDown();
            throw new IllegalStateException("stale activation completed late");
        }

        @Override
        public long getTime() {
            return -1;
        }

        @Override
        public void onStop() {
            stopCalls.incrementAndGet();
        }
    }

    private static final class ObservingMovementController extends MovementController {
        private final CountDownLatch watchedFinishAttempt = new CountDownLatch(1);
        private Movement watched;

        private void watch(Movement movement) {
            watched = movement;
        }

        @Override
        public boolean finishCurrentMovement(Movement expectedMovement, long expectedActivationId) {
            try {
                return super.finishCurrentMovement(expectedMovement, expectedActivationId);
            } finally {
                if (expectedMovement == watched) watchedFinishAttempt.countDown();
            }
        }
    }

    private static final class ImmediateMovement extends Movement {
        @Override
        public void init() {}

        @Override
        public void onTick() {
            setFinished(true);
        }

        @Override
        public long getTime() {
            return -1;
        }

        @Override
        public void onStop() {}
    }

    private static final class RecordingMovement extends Movement {
        private boolean throwOnTick;
        private int stopCalls;

        @Override
        public void init() {}

        @Override
        public void onTick() {
            if (throwOnTick) throw new IllegalStateException("stale task failure");
        }

        @Override
        public long getTime() {
            return -1;
        }

        @Override
        public void onStop() {
            stopCalls++;
        }
    }
}
