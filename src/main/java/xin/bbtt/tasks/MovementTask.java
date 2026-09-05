package xin.bbtt.tasks;

import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.movement.MovementController;

public class MovementTask implements Runnable {
    private final Movement movement;
    private final MovementController controller;
    private final long activationId;
    private int elapsedTicks = 0;

    public MovementTask(Movement movement, MovementController controller, long activationId) {
        this.movement = movement;
        this.controller = controller;
        this.activationId = activationId;
    }

    @Override
    public void run() {
        if (Thread.currentThread().isInterrupted() || controller.isPaused()) {
            return;
        }

        try {
            this.movement.onTick();
            elapsedTicks++;

            long timeLimit = this.movement.getTime();
            if (this.movement.isFinished() || (timeLimit >= 0 && elapsedTicks * 50L >= timeLimit)) {
                controller.finishCurrentMovement(movement, activationId);
            }
        } catch (Exception e) {
            MovementSync.getLogger().error("Failed to run onTick", e);
            controller.finishCurrentMovement(movement, activationId);
        }
    }
}
