package xin.bbtt.physics;

import org.joml.Vector3d;

/** Stateless key steering shared by live navigation and trajectory previews. */
public final class JumpInputController {
    public record Command(PlayerInput input, float yaw) {}

    private JumpInputController() {}

    /** Coast toward the resting position on slippery ground; reverse keys only to brake overshoot. */
    public static Command groundArrival(Vector3d position, Vector3d target, Vector3d velocity,
            double slipperiness) {
        double drag = Math.min(0.999, slipperiness * 0.91);
        double steeringX = target.x - position.x - velocity.x / (1 - drag);
        double steeringZ = target.z - position.z - velocity.z / (1 - drag);
        float yaw = (float) Math.toDegrees(Math.atan2(-steeringX, steeringZ));
        boolean moving = Math.hypot(steeringX, steeringZ) > 0.12;
        return new Command(new PlayerInput(moving, false, false, false,
                false, false, false), yaw);
    }

    public static Command toward(
            Vector3d position, Vector3d target, Vector3d velocity,
            boolean grounded, boolean gapJump) {
        double differenceX = target.x - position.x;
        double differenceZ = target.z - position.z;
        double distance = Math.hypot(differenceX, differenceZ);
        // Counter residual momentum rather than snapping position or overwriting velocity.
        double steeringX = differenceX - velocity.x * 2;
        double steeringZ = differenceZ - velocity.z * 2;
        boolean moving = distance > 0.08 || Math.hypot(velocity.x, velocity.z) > 0.04;
        float yaw = (float) Math.toDegrees(Math.atan2(-steeringX, steeringZ));
        boolean jump = grounded && (target.y - position.y > 0.6
                || (gapJump && distance > 0.8));
        return new Command(new PlayerInput(moving, false, false, false,
                jump, false, gapJump && distance > 0.8), yaw);
    }
}
