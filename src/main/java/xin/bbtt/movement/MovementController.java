package xin.bbtt.movement;

import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.tasks.MovementTask;

import java.util.Deque;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class MovementController {
    private final Deque<Movement> movements = new ConcurrentLinkedDeque<>();
    private final AtomicBoolean isExecuting = new AtomicBoolean(false);
    private final AtomicBoolean isPaused = new AtomicBoolean(false);

    private final Object stateLock = new Object();

    private ScheduledFuture<?> currentTaskFuture = null;
    private Movement currentMovement = null;

    public void addMovement(Movement movement) {
        movements.addLast(movement);
        tryExecuteNext();
    }

    public void insertMovement(Movement movement) {
        synchronized (stateLock) {
            if (currentMovement != null) {
                Movement pausedMovement = currentMovement;
                stopCurrentMovement();
                movements.addFirst(pausedMovement);
            }
            movements.addFirst(movement);
        }
        tryExecuteNext();
    }

    public void pause() {
        if (!isPaused.compareAndSet(false, true)) return;

        synchronized (stateLock) {
            if (currentMovement == null) return;
            try {
                currentMovement.onPause();
            } catch (Exception e) {
                MovementSync.Instance.getLogger().error(LangManager.get("movementsync.movement.error.pause"), e);
            }
        }
    }

    public void resume() {
        if (!isPaused.compareAndSet(true, false)) return;

        boolean shouldExecuteNext = false;
        synchronized (stateLock) {
            if (currentMovement != null) {
                try {
                    currentMovement.onResume();
                } catch (Exception e) {
                    MovementSync.Instance.getLogger().error(LangManager.get("movementsync.movement.error.resume"), e);
                }
            } else {
                shouldExecuteNext = true;
            }
        }
        
        if (shouldExecuteNext) {
            tryExecuteNext();
        }
    }

    public boolean isPaused() {
        return isPaused.get();
    }

    public void cancelAll() {
        synchronized (stateLock) {
            movements.clear();
            stopCurrentMovement();
        }
    }

    @SuppressWarnings("unused")
    public boolean hasMovement() {
        synchronized (stateLock) {
            return !movements.isEmpty() || currentMovement != null;
        }
    }

    private void tryExecuteNext() {
        if (isExecuting.get()) return;
        if (movements.isEmpty()) return;
        if (isPaused.get()) return;

        if (isExecuting.compareAndSet(false, true)) {
            doNext();
        }
    }

    private void doNext() {
        synchronized (stateLock) {
            currentMovement = movements.pollFirst();

            if (currentMovement == null) {
                isExecuting.set(false);
                return;
            }

            if (MovementSync.Instance.movementService == null || MovementSync.Instance.movementService.isShutdown()) {
                MovementSync.Instance.movementService = Executors.newScheduledThreadPool(1);
            }

            try {
                currentMovement.init();
                MovementTask task = new MovementTask(currentMovement, this);

                currentTaskFuture = MovementSync.Instance.movementService.scheduleAtFixedRate(
                        task,
                        0L,
                        50L,
                        TimeUnit.MILLISECONDS
                );
            } catch (Exception e) {
                MovementSync.Instance.getLogger().error(LangManager.get("movementsync.movement.error.run"), e);
                isExecuting.set(false);
            }
        }
        
        if (!isExecuting.get() && !isPaused.get()) {
            tryExecuteNext();
        }
    }

    public void finishCurrentMovement() {
        boolean shouldExecuteNext;
        synchronized (stateLock) {
            stopCurrentMovement();
            shouldExecuteNext = !isPaused.get();
        }
        if (shouldExecuteNext) {
            tryExecuteNext();
        }
    }

    private void stopCurrentMovement() {
        if (currentTaskFuture != null) {
            currentTaskFuture.cancel(true);
            currentTaskFuture = null;
        }

        if (currentMovement != null) {
            try {
                currentMovement.onStop();
            } catch (Exception e) {
                MovementSync.Instance.getLogger().error(LangManager.get("movementsync.movement.error.stop"), e);
            }
            currentMovement = null;
        }

        isExecuting.set(false);
    }
}
