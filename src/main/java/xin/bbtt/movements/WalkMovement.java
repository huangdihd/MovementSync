package xin.bbtt.movements;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;

public class WalkMovement extends Movement {
    private final Vector3d velocity;
    private final long time;

    public WalkMovement(Vector3d velocity, long time){
        this.velocity = velocity;
        this.time = time;
    }

    @Override
    public void init() {
        MovementSync.INSTANCE.velocity.updateAndGet(p -> new Vector3d(p).add(velocity));
    }

    @Override
    public void onTick() {

    }

    @Override
    public long getTime() {
        return time;
    }

    @Override
    public void onStop() {
        MovementSync.INSTANCE.velocity.updateAndGet(p -> new Vector3d(p).sub(velocity));
    }
}
