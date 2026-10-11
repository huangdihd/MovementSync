package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.tasks.VerticalCollisionResolver;
import xin.bbtt.world.World;

import static xin.bbtt.MovementSync.gravitationalAcceleration;
import static xin.bbtt.MovementSync.terminalVelocity;

public final class WalkingPhysics implements PhysicsStepper {
    private static final double PLAYER_HALF_WIDTH = 0.299;
    private static final double MAX_STEP_HEIGHT =
        xin.bbtt.pathfinding.StandablePositionResolver.MAX_STEP_RISE;
    private static final double STEP_EPSILON = 1.0e-7;
    private static final double MIN_MOVEMENT = 0.003;
    // BlockEntry/BlockState currently expose no friction; use vanilla's default.
    private static final double BLOCK_FRICTION = 0.6;
    private final PlayerInput input;
    private final float yaw;
    private final boolean jumpAllowed;

    public WalkingPhysics() {
        this(PlayerInput.NONE, 0, true);
    }

    public WalkingPhysics(PlayerInput input, float yaw) {
        this(input, yaw, true);
    }

    public WalkingPhysics(PlayerInput input, float yaw, boolean jumpAllowed) {
        this.input = java.util.Objects.requireNonNull(input, "input");
        this.yaw = yaw;
        this.jumpAllowed = jumpAllowed;
    }

    @Override
    public PhysicsResult step(World world, PhysicsState state, double halfWidth, double height) {
        Vector3d position = state.position();
        Vector3d velocity = state.velocity();
        Vector3d displacement = new Vector3d();
        velocity.set(snapToZero(velocity.x), snapToZero(velocity.y), snapToZero(velocity.z));

        final double tickStartY = position.y;
        boolean groundedAtTickStart = state.onGround();
        double yawRadians = Math.toRadians(yaw);
        if (groundedAtTickStart && input.jump() && jumpAllowed) {
            velocity.y = MovementSync.jumpVelocity;
            if (input.sprint()) {
                velocity.x -= Math.sin(yawRadians) * 0.2;
                velocity.z += Math.cos(yawRadians) * 0.2;
            }
        }
        accelerate(velocity, groundedAtTickStart, yawRadians);

        /*
         * Match the client tick order: move with this tick's accumulated
         * velocity (including input), then apply gravity and vertical drag for
         * the next tick.  Applying gravity before the move shortens the first
         * jump tick by about 0.087 blocks and makes every jump trajectory
         * diverge from the server's movement prediction.
         */
        displacement.x = velocity.x;
        displacement.z = velocity.z;
        displacement.y = velocity.y;

        // Vanilla clips downward travel before horizontal travel. At an edge the
        // last supported tick therefore stays grounded even after moving past it.
        boolean downwardCollision = false;
        if (displacement.y < 0) {
            double desiredY = position.y + displacement.y;
            double resolvedY = VerticalCollisionResolver.resolveDownwardY(
                    world, position, halfWidth, displacement.y);
            downwardCollision = resolvedY > desiredY + 1.0e-9;
            position.y = resolvedY;
            if (downwardCollision) velocity.y = 0;
            displacement.y = 0;
        }

        boolean collidedX = false;
        boolean collidedZ = false;

        if (displacement.x != 0) {
            Vector3d testPosX = new Vector3d(position.x + displacement.x, position.y, position.z);
            if (isBoxColliding(world, testPosX, halfWidth, height)) {
                if (!tryStepUp(world, state.onGround(), halfWidth, height, position, testPosX.x, testPosX.z, tickStartY)) {
                    displacement.x = 0;
                    velocity.x = 0;
                    collidedX = true;
                }
            } else {
                position.x += displacement.x;
            }
        }

        if (displacement.z != 0) {
            Vector3d testPosZ = new Vector3d(position.x, position.y, position.z + displacement.z);
            if (isBoxColliding(world, testPosZ, halfWidth, height)) {
                if (!tryStepUp(world, state.onGround(), halfWidth, height, position, testPosZ.x, testPosZ.z, tickStartY)) {
                    displacement.z = 0;
                    velocity.z = 0;
                    collidedZ = true;
                }
            } else {
                position.z += displacement.z;
            }
        }

        if (displacement.y > 0) {
            Vector3d testPosY = new Vector3d(position.x, position.y + displacement.y, position.z);
            if (isBoxColliding(world, testPosY, halfWidth, height)) {
                displacement.y = 0;
                velocity.y = 0;
            }
        }

        if (displacement.y < 0) {
            double desiredY = position.y + displacement.y;
            double resolvedY = VerticalCollisionResolver.resolveDownwardY(
                world, position, 0.299, displacement.y);
            position.y = resolvedY;
            if (resolvedY > desiredY + 1.0e-9) velocity.y = 0;
        } else {
            position.y += displacement.y;
        }

        // Ground contact cancels downward movement, not next tick's gravity.
        // Retain the falling velocity so leaving an edge does not hover for a tick.
        velocity.y = Math.max(
            (velocity.y + gravitationalAcceleration.y) * MovementSync.verticalDrag,
            terminalVelocity);
        double horizontalDrag = groundedAtTickStart ? BLOCK_FRICTION * 0.91 : 0.91;
        velocity.x *= horizontalDrag;
        velocity.z *= horizontalDrag;
        return new PhysicsResult(position, velocity, collidedX, collidedZ,
                downwardCollision || (state.onGround() && state.velocity().y == 0
                        && !input.jump() && world.isOnGround(position)));
    }

    private static double snapToZero(double component) {
        return Math.abs(component) < MIN_MOVEMENT ? 0 : component;
    }

    private void accelerate(Vector3d velocity, boolean onGround, double yawRadians) {
        double forward = (input.forward() ? 1 : 0) - (input.backward() ? 1 : 0);
        double strafe = (input.left() ? 1 : 0) - (input.right() ? 1 : 0);
        double length = Math.hypot(forward, strafe);
        if (length == 0) return;

        double keyScale = 0.98 * (input.sneak() ? 0.3 : 1) / Math.max(1, length);
        forward *= keyScale;
        strafe *= keyScale;
        double acceleration = onGround
            ? 0.1 * (input.sprint() ? 1.3 : 1) * 0.21600002
                / (BLOCK_FRICTION * BLOCK_FRICTION * BLOCK_FRICTION)
            : (input.sprint() ? 0.026 : 0.02);
        double sine = Math.sin(yawRadians);
        double cosine = Math.cos(yawRadians);
        velocity.x += (strafe * cosine - forward * sine) * acceleration;
        velocity.z += (forward * cosine + strafe * sine) * acceleration;
    }

    private static boolean isBoxColliding(
            World world, Vector3d position, double halfWidth, double height) {
        return world.isBoxColliding(
            position.x - halfWidth, position.y, position.z - halfWidth,
            position.x + halfWidth, position.y + height, position.z + halfWidth);
    }

    /**
     * Applies vanilla-style automatic stepping for grounded horizontal motion.
     * The candidate support height comes from the exact block-state collision
     * boxes, so path blocks, slabs, stairs, trapdoors, and directional shapes
     * keep their real heights instead of being treated as whole cubes.
     */
    private static boolean tryStepUp(
            World world, boolean onGround, double halfWidth, double height,
            Vector3d position,
            double candidateX,
            double candidateZ,
            double tickStartY) {
        if (!onGround) return false;

        java.util.OptionalDouble support = world.findHighestCollisionTop(
            candidateX - PLAYER_HALF_WIDTH, position.y + STEP_EPSILON, candidateZ - PLAYER_HALF_WIDTH,
            candidateX + PLAYER_HALF_WIDTH, tickStartY + MAX_STEP_HEIGHT, candidateZ + PLAYER_HALF_WIDTH
        );
        if (support.isEmpty()) return false;

        Vector3d stepped = new Vector3d(candidateX, support.getAsDouble(), candidateZ);
        if (isBoxColliding(world, stepped, halfWidth, height)) return false;
        position.set(stepped);
        return true;
    }

}
