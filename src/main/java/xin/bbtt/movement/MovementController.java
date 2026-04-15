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

    /**
     * "IMMEDIATE" means interrupting the current thread NOW.
     */
    public void insertMovement(Movement movement) {
        synchronized (stateLock) {
            // Kill current task forcefully
            if (currentTaskFuture != null) {
                currentTaskFuture.cancel(true);
                currentTaskFuture = null;
            }
            
            if (currentMovement != null) {
                try {
                    currentMovement.onStop();
                } catch (Exception ignored) {}
                // Put it back to the head so it resumes after the inserted one
                movements.addFirst(currentMovement);
                currentMovement = null;
            }
            
            // Insert the new one at the very front
            movements.addFirst(movement);
            isExecuting.set(false);
        }
        // Force start the new task in this thread
        tryExecuteNext();
    }

    public void pause() {
        if (!isPaused.compareAndSet(false, true)) return;
        synchronized (stateLock) {
            if (currentMovement != null) {
                try { currentMovement.onPause(); } catch (Exception ignored) {}
            }
        }
    }

    public void resume() {
        if (!isPaused.compareAndSet(true, false)) return;
        synchronized (stateLock) {
            if (currentMovement != null) {
                try { currentMovement.onResume(); } catch (Exception ignored) {}
            }
        }
        tryExecuteNext();
    }

    public boolean isPaused() {
        return isPaused.get();
    }

    public Movement getCurrentMovement() {
        synchronized (stateLock) {
            return currentMovement;
        }
    }

    public void cancelAll() {
        synchronized (stateLock) {
            movements.clear();
            stopCurrent();
        }
    }

    public void finishCurrentMovement() {
        synchronized (stateLock) {
            stopCurrent();
        }
        tryExecuteNext();
    }

    private void stopCurrent() {
        if (currentTaskFuture != null) {
            currentTaskFuture.cancel(true);
            currentTaskFuture = null;
        }
        if (currentMovement != null) {
            try { currentMovement.onStop(); } catch (Exception ignored) {}
            currentMovement = null;
        }
        isExecuting.set(false);
    }

    private void tryExecuteNext() {
        if (isPaused.get()) return;
        if (movements.isEmpty()) return;
        
        // Ensure atomic handover
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
                currentTaskFuture = MovementSync.Instance.movementService.scheduleAtFixedRate(task, 0L, 50L, TimeUnit.MILLISECONDS);
            } catch (Exception e) {
                MovementSync.Instance.getLogger().error(LangManager.get("movementsync.movement.error.run"), e);
                isExecuting.set(false);
                tryExecuteNext();
            }
        }
    }
}
