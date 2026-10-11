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

    @Override
    public PhysicsResult step(World world, PhysicsState state, double halfWidth, double height) {
        Vector3d position = state.position();
        Vector3d velocity = state.velocity();
        Vector3d displacement = new Vector3d();

        if (state.onGround() && velocity.y < 0) {
            velocity.y = 0;
        }

        final double tickStartY = position.y;

        /*
         * Match the client tick order: move with the velocity that was already
         * accumulated for this tick, then apply gravity and vertical drag for
         * the next tick.  Applying gravity before the move shortens the first
         * jump tick by about 0.087 blocks and makes every jump trajectory
         * diverge from the server's movement prediction.
         */
        displacement.x = velocity.x;
        displacement.z = velocity.z;
        boolean groundedAtTickStart = state.onGround();
        boolean hasUpwardImpulse = velocity.y > 0;
        if (!groundedAtTickStart || hasUpwardImpulse) {
            displacement.y = velocity.y;
            velocity.y = Math.max(
                    (velocity.y + gravitationalAcceleration.y) * MovementSync.verticalDrag,
                    terminalVelocity);
        } else {
            displacement.y = 0;
            if (velocity.y < 0) velocity.y = 0;
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

        return new PhysicsResult(position, velocity, collidedX, collidedZ);
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
