package xin.bbtt.movements;

import xin.bbtt.movement.Movement;

public class ActionMovement extends Movement {
    private final Runnable action;
    private final long delay;
    private int elapsed = 0;

    public ActionMovement(Runnable action) {
        this(action, 0);
    }

    public ActionMovement(Runnable action, long delay) {
        this.action = action;
        this.delay = delay;
    }

    @Override
    public void init() {}

    @Override
    public void onTick() {
        if (elapsed * 50 >= delay) {
            action.run();
            setFinished(true);
        }
        elapsed++;
    }

    @Override
    public long getTime() {
        return -1;
    }

    @Override
    public void onStop() {}
}
