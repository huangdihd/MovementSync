package xin.bbtt.movements;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;


public class LookAtMovement extends Movement {
    public final Vector3d target;
    public final float targetYaw;
    public final float targetPitch;
    public float vy, vp;
    private long time = 0;

    static float wrapDegrees(float deg) {
        deg = deg % 360.0f;
        if (deg >= 180.0f) deg -= 360.0f;
        if (deg < -180.0f) deg += 360.0f;
        return deg;
    }


    public LookAtMovement(Vector3d target) {
        this.target = target;
        Vector3d delta = new Vector3d(target).sub(MovementSync.INSTANCE.getHeadPosition());

        double dx = delta.x;
        double dy = delta.y;
        double dz = delta.z;

        targetYaw = (float) Math.toDegrees(Math.atan2(dz, dx)) - 90;
        targetPitch = (float) Math.toDegrees(-Math.atan2(dy, Math.sqrt(dx * dx + dz * dz)));
    }

    @Override
    public void init() {
        float dy = wrapDegrees(targetYaw - MovementSync.INSTANCE.yaw.get());
        float dp = wrapDegrees(targetPitch - MovementSync.INSTANCE.pitch.get());
        this.time = (long) (Math.sqrt(dy * dy + dp * dp) / 90 * 1000);
        if (this.time <= 0) this.time = 1; // To avoid divide 0
        this.vy = dy / this.time * 50;
        this.vp = dp / this.time * 50;
    }


    @Override
    public void onTick() {
        MovementSync.INSTANCE.yaw.updateAndGet(yaw -> yaw + this.vy);
        MovementSync.INSTANCE.pitch.updateAndGet(pitch -> pitch + this.vp);
    }

    @Override
    public long getTime() {
        return time;
    }

    @Override
    public void onStop() {
        MovementSync.INSTANCE.yaw.set(targetYaw);
        MovementSync.INSTANCE.pitch.set(targetPitch);
    }
}
