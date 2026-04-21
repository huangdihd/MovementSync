package xin.bbtt.movements;

import org.joml.Vector3d;
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
        if (!MovementSync.INSTANCE.onGround.get()) return;
        MovementSync.getLogger().info(LangManager.get("movementsync.movement.jump"));
        MovementSync.INSTANCE.velocity.updateAndGet(p -> new Vector3d(p).add(new Vector3d(0, 0.42, 0)));
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
