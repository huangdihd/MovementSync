package xin.bbtt.physics;

import org.joml.Vector3d;

/** Stateless key steering shared by live navigation and trajectory previews. */
public final class JumpInputController {
    public record Command(PlayerInput input, float yaw) {}

    private JumpInputController() {}

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
