package xin.bbtt.movements;

import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.movement.Movement;

public class JumpMovement extends Movement {
    long time;

    public JumpMovement(long time) {
        this.time = time;
    }

    public JumpMovement() {
        this(0);
    }

    @Override
    public void init() {
        MovementSync.getLogger().info(LangManager.get("movementsync.movement.jump"));
        MovementSync.INSTANCE.jump();
    }

    @Override
    public void onTick() {

    }

    @Override
    public long getTime() {
        return 0;
    }

    @Override
    public void onStop() {
    }
}
