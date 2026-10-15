package xin.bbtt.movements;

import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.movement.Movement;

/** Waits for the falling part of a jump before requesting a glide. */
public final class ElytraTakeoffMovement extends Movement {
    @Override
    public void init() {
        if (MovementSync.INSTANCE.onGround.get()) MovementSync.INSTANCE.jump();
    }

    @Override
    public void onTick() {
        if (isFinished()) return;
        MovementSync movementSync = MovementSync.INSTANCE;
        if (movementSync.onGround.get() || movementSync.velocity.get().y > 0) return;

        boolean requested = movementSync.startGliding();
        setFinished(true);
        MovementSync.getLogger().info(LangManager.get(requested
                ? "movementsync.command.elytra.start.requested"
                : "movementsync.command.elytra.start.failed"));
    }

    @Override
    public long getTime() {
        return 3000L;
    }

    @Override
    public void onStop() {
        // The controller calls onStop for both time limits and cancellation.
        if (isFinished()) return;
        setFinished(true);
        MovementSync.getLogger().info(LangManager.get("movementsync.command.elytra.start.failed"));
    }
}
