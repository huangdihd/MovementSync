package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.tasks.VerticalCollisionResolver;
import xin.bbtt.world.World;

import static xin.bbtt.MovementSync.terminalVelocity;

public final class WalkingPhysics implements PhysicsStepper {
    private static final double PLAYER_HALF_WIDTH = (double) 0.6f / 2;
    private static final double MAX_STEP_HEIGHT =
        xin.bbtt.pathfinding.StandablePositionResolver.MAX_STEP_RISE;
    private static final double STEP_EPSILON = 1.0e-7;
    private static final double MIN_MOVEMENT = 0.003;
    private final PlayerInput input;
    private final float yaw;
    private final boolean jumpAllowed;
    private final MovementParameters parameters;

    public WalkingPhysics() {
        this(PlayerInput.NONE, 0, true);
    }

    public WalkingPhysics(PlayerInput input, float yaw) {
        this(input, yaw, true);
    }

    public WalkingPhysics(PlayerInput input, float yaw, boolean jumpAllowed) {
        this(input, yaw, jumpAllowed, MovementParameters.DEFAULT);
    }

    public WalkingPhysics(PlayerInput input, float yaw, boolean jumpAllowed, MovementParameters parameters) {
        this.parameters = java.util.Objects.requireNonNull(parameters, "parameters");
        this.input = java.util.Objects.requireNonNull(input, "input");
        this.yaw = yaw;
        this.jumpAllowed = jumpAllowed;
    }

    @Override
    public PhysicsResult step(World world, PhysicsState state, double halfWidth, double height) {
        Vector3d position = state.position();
        Vector3d velocity = state.velocity();
        double fallDistance = parameters.slowFalling() || parameters.levitationAmplifier() >= 0 ? 0 : state.fallDistance();
        Vector3d displacement = new Vector3d();
        EnvironmentalPhysics.Environment environment = EnvironmentalPhysics.sample(world, position, halfWidth, height);
        double slipperiness = EnvironmentalPhysics.supportProperties(world, position).slipperiness();
        boolean water = environment.waterHeight() > 0;
        boolean lava = !water && environment.lavaHeight() > 0;
        boolean fluid = water || lava;
        boolean slowDown = state.pose().shouldSlowDown(false, water, input.sneak());
        boolean sprint = input.sprint() && !state.pose().shouldSlowDown(false, water, false);
        addFluidPush(velocity, environment.waterPush());
        addFluidPush(velocity, environment.lavaPush());
        // Since 1.21.5 the horizontal threshold applies to the vector, not each axis.
        if (velocity.x * velocity.x + velocity.z * velocity.z < MIN_MOVEMENT * MIN_MOVEMENT) {
            velocity.x = 0;
            velocity.z = 0;
        }
        velocity.y = snapToZero(velocity.y);
        if (water && input.sneak()) velocity.y -= (double) 0.04f;

        final double tickStartY = position.y;
        boolean groundedAtTickStart = state.onGround();
        double yawRadians = Math.toRadians(yaw);
        double fluidDepth = water ? environment.waterHeight() : environment.lavaHeight();
        boolean swimJump = fluid && (!groundedAtTickStart || fluidDepth > 0.4);
        if (input.jump() && swimJump) velocity.y += (double) 0.04f;
        if (groundedAtTickStart && input.jump() && jumpAllowed && !swimJump) {
            velocity.y = parameters.jumpVelocity(EnvironmentalPhysics.jumpMultiplier(world, position));
            if (sprint && !fluid) {
                velocity.x -= Math.sin(yawRadians) * 0.2;
                velocity.z += Math.cos(yawRadians) * 0.2;
            }
        }
        accelerate(velocity, groundedAtTickStart, yawRadians, fluid, slipperiness, slowDown, sprint);
        if (!fluid && environment.climbing()) {
            fallDistance = 0;
            velocity.x = Math.max(-(double) 0.15f, Math.min((double) 0.15f, velocity.x));
            velocity.z = Math.max(-(double) 0.15f, Math.min((double) 0.15f, velocity.z));
            velocity.y = Math.max(-(double) 0.15f, velocity.y);
            if (input.sneak() && !environment.scaffolding() && velocity.y < 0) velocity.y = 0;
        }

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
        Vector3d stuckMultiplier = EnvironmentalPhysics.stuckMultiplier(world, position, halfWidth, height);
        if (stuckMultiplier.x != 1 || stuckMultiplier.y != 1 || stuckMultiplier.z != 1) {
            fallDistance = 0;
            displacement.mul(stuckMultiplier);
            velocity.zero();
        }

        if (input.sneak() && groundedAtTickStart && displacement.y <= 0) {
            protectSneakingEdge(world, position, displacement, halfWidth, height);
        }

        // Vanilla clips downward travel before horizontal travel. At an edge the
        // last supported tick therefore stays grounded even after moving past it.
        boolean downwardCollision = false;
        if (displacement.y < 0) {
            double desiredY = position.y + displacement.y;
            double resolvedY = VerticalCollisionResolver.resolveDownwardY(
                    world, position, halfWidth, displacement.y, fallDistance);
            downwardCollision = resolvedY > desiredY + 1.0e-9;
            position.y = resolvedY;
            if (downwardCollision) {
                double bounce = EnvironmentalPhysics.landingProperties(world, position).bounce();
                velocity.y = input.sneak() ? 0 : -velocity.y * bounce;
            }
            displacement.y = 0;
        }

        // Resolve upward travel before horizontal motion too: jumping onto a full
        // block must test the raised body, and a ceiling clips to its actual face.
        if (displacement.y > 0) {
            double resolvedY = clipMovement(world, position, halfWidth, height, 1, displacement.y);
            position.y += resolvedY;
            if (resolvedY != displacement.y) velocity.y = 0;
        }

        boolean zFirst = Math.abs(displacement.x) < Math.abs(displacement.z);
        boolean collidedX;
        boolean collidedZ;
        boolean canStep = groundedAtTickStart || downwardCollision;
        if (zFirst) {
            collidedZ = moveHorizontal(world, position, velocity, halfWidth, height,
                2, displacement.z, canStep, tickStartY);
            collidedX = moveHorizontal(world, position, velocity, halfWidth, height,
                0, displacement.x, canStep, tickStartY);
        } else {
            collidedX = moveHorizontal(world, position, velocity, halfWidth, height,
                0, displacement.x, canStep, tickStartY);
            collidedZ = moveHorizontal(world, position, velocity, halfWidth, height,
                2, displacement.z, canStep, tickStartY);
        }

        fallDistance = downwardCollision ? 0 : fallDistance + Math.max(0, tickStartY - position.y);
        // makeStuckInBlock resets accumulated fall distance at the contact callback.
        Vector3d contactMultiplier = EnvironmentalPhysics.stuckMultiplier(world, position, halfWidth, height);
        if (contactMultiplier.x != 1 || contactMultiplier.y != 1 || contactMultiplier.z != 1
                || water) fallDistance = 0;

        boolean climbingAfterMove = EnvironmentalPhysics.sample(world, position, halfWidth, height).climbing();
        double velocityMultiplier = EnvironmentalPhysics.velocityMultiplier(world, position);
        velocity.x *= velocityMultiplier;
        velocity.z *= velocityMultiplier;
        BlockMovementProperties landing = EnvironmentalPhysics.landingProperties(world, position);
        if (downwardCollision && landing.stickyStepping() && !input.sneak() && Math.abs(velocity.y) < 0.1) {
            double scale = 0.4 + Math.abs(velocity.y) * 0.2;
            velocity.x *= scale;
            velocity.z *= scale;
        }
        if (fluid) {
            if (water) {
                if ((collidedX || collidedZ) && climbingAfterMove) velocity.y = 0.2;
                velocity.mul((double) 0.8f);
                velocity.y = fluidGravity(velocity.y, parameters.effectiveGravity(state.velocity().y), state.velocity().y <= 0);
            } else {
                if (environment.lavaHeight() <= 0.4) {
                    velocity.mul(0.5, (double) 0.8f, 0.5);
                    velocity.y = fluidGravity(velocity.y, parameters.effectiveGravity(state.velocity().y), state.velocity().y <= 0);
                } else {
                    velocity.mul(0.5);
                }
                velocity.y -= parameters.effectiveGravity(state.velocity().y) / 4;
            }
            if (collidedX || collidedZ) {
                Vector3d exit = new Vector3d(position).add(velocity.x,
                        velocity.y + (double) 0.6f - position.y + tickStartY, velocity.z);
                EnvironmentalPhysics.Environment exitEnvironment = EnvironmentalPhysics.sample(world, exit, halfWidth, height);
                if (!isBoxColliding(world, exit, halfWidth, height)
                        && exitEnvironment.waterHeight() == 0 && exitEnvironment.lavaHeight() == 0) {
                    velocity.y = (double) 0.3f;
                }
            }
            EnvironmentalPhysics.applyContactVelocity(world, position, velocity, halfWidth, height, downwardCollision);
            return new PhysicsResult(position, velocity, collidedX, collidedZ, downwardCollision, fallDistance,
                    state.pose().afterTravel(world, position, false, false, input.sneak()));
        }
        if ((collidedX || collidedZ || input.jump()) && climbingAfterMove) velocity.y = 0.2;

        // Ground contact cancels downward movement, not next tick's gravity.
        // Retain the falling velocity so leaving an edge does not hover for a tick.
        // LivingEntity.travelMidAir: missing client chunks replace gravity, after movement.
        if (parameters.levitationAmplifier() >= 0
                || world.chunkLoaded((int) Math.floor(position.x) >> 4, (int) Math.floor(position.z) >> 4)) {
            velocity.y = Math.max(
                parameters.verticalVelocityAfterForces(velocity.y) * MovementSync.verticalDrag,
                terminalVelocity);
        } else {
            velocity.y = (position.y > World.getMinWorldY() ? -0.1 : 0) * MovementSync.verticalDrag;
        }
        double horizontalDrag = groundedAtTickStart ? slipperiness * 0.91 : 0.91;
        velocity.x *= horizontalDrag;
        velocity.z *= horizontalDrag;
        EnvironmentalPhysics.applyContactVelocity(world, position, velocity, halfWidth, height, downwardCollision);
        return new PhysicsResult(position, velocity, collidedX, collidedZ,
                downwardCollision || (state.onGround() && state.velocity().y == 0
                        && !input.jump() && world.isOnGround(position)), fallDistance,
                state.pose().afterTravel(world, position, false, false, input.sneak()));
    }

    private boolean moveHorizontal(
            World world, Vector3d position, Vector3d velocity, double halfWidth, double height,
            int axis, double movement, boolean canStep, double tickStartY) {
        if (movement == 0) return false;
        double clipped = clipMovement(world, position, halfWidth, height, axis, movement);
        if (clipped == movement) {
            position.setComponent(axis, position.get(axis) + movement);
            return false;
        }
        double candidateX = position.x + (axis == 0 ? movement : 0);
        double candidateZ = position.z + (axis == 2 ? movement : 0);
        if (tryStepUp(world, canStep, halfWidth, height, position, candidateX, candidateZ, tickStartY)) {
            return false;
        }
        position.setComponent(axis, position.get(axis) + clipped);
        velocity.setComponent(axis, 0);
        return true;
    }

    static double clipMovement(
            World world, Vector3d position, double halfWidth, double height, int axis, double movement) {
        if (!isSweepColliding(world, position, halfWidth, height, axis, movement)) return movement;
        // World exposes overlap queries rather than collision boxes. A swept AABB
        // gives a monotonic predicate, including when the endpoint passed a thin wall.
        double clear = 0;
        double blocked = movement;
        for (int iteration = 0; iteration < 52; iteration++) {
            double middle = (clear + blocked) * 0.5;
            if (middle == clear || middle == blocked) break;
            if (isSweepColliding(world, position, halfWidth, height, axis, middle)) blocked = middle;
            else clear = middle;
        }
        return clear;
    }

    private static boolean isSweepColliding(
            World world, Vector3d position, double halfWidth, double height, int axis, double movement) {
        double deltaX = axis == 0 ? movement : 0;
        double deltaY = axis == 1 ? movement : 0;
        double deltaZ = axis == 2 ? movement : 0;
        // Form the endpoint center before its bounds, just as moveHorizontal does.
        // Different addition order can round a touching face into the wall and
        // make the next upward sweep start inside an obstacle.
        return world.isBoxColliding(
            (position.x + Math.min(0, deltaX)) - halfWidth, position.y + Math.min(0, deltaY),
            (position.z + Math.min(0, deltaZ)) - halfWidth,
            (position.x + Math.max(0, deltaX)) + halfWidth,
            (position.y + Math.max(0, deltaY)) + height,
            (position.z + Math.max(0, deltaZ)) + halfWidth);
    }

    private static void protectSneakingEdge(
            World world, Vector3d position, Vector3d displacement, double halfWidth, double height) {
        // Vanilla backs off each axis, then the diagonal, in 0.05-block increments.
        // This adjusts travel, not momentum: edge protection is not a wall collision.
        while (displacement.x != 0 && !hasSneakingSupport(
                world, position, displacement.x, 0, halfWidth, height)) {
            displacement.x = backOffEdge(displacement.x);
        }
        while (displacement.z != 0 && !hasSneakingSupport(
                world, position, 0, displacement.z, halfWidth, height)) {
            displacement.z = backOffEdge(displacement.z);
        }
        while (displacement.x != 0 && displacement.z != 0 && !hasSneakingSupport(
                world, position, displacement.x, displacement.z, halfWidth, height)) {
            displacement.x = backOffEdge(displacement.x);
            displacement.z = backOffEdge(displacement.z);
        }
    }

    private static boolean hasSneakingSupport(
            World world, Vector3d position, double deltaX, double deltaZ,
            double halfWidth, double height) {
        return isBoxColliding(world,
            new Vector3d(position.x + deltaX, position.y - MAX_STEP_HEIGHT, position.z + deltaZ),
            halfWidth, height);
    }

    private static double backOffEdge(double movement) {
        if (Math.abs(movement) <= 0.05) return 0;
        return movement - Math.copySign(0.05, movement);
    }

    private static double snapToZero(double component) {
        return Math.abs(component) < MIN_MOVEMENT ? 0 : component;
    }

    private static void addFluidPush(Vector3d velocity, Vector3d push) {
        if (push.lengthSquared() == 0) return;
        Vector3d applied = new Vector3d(push);
        if (Math.abs(velocity.x) < 0.003 && Math.abs(velocity.z) < 0.003 && applied.length() < 0.0045000000000000005) {
            applied.normalize(0.0045000000000000005);
        }
        velocity.add(applied);
    }

    private static double fluidGravity(double verticalVelocity, double gravity, boolean falling) {
        // LivingEntity.applyFluidMovingSpeed avoids tiny positive motion at low gravity.
        if (gravity == 0) return verticalVelocity;
        if (falling && Math.abs(verticalVelocity - 0.005) >= 0.003
                && Math.abs(verticalVelocity - gravity / 16) < 0.003) return -0.003;
        return verticalVelocity - gravity / 16;
    }

    private void accelerate(Vector3d velocity, boolean onGround, double yawRadians, boolean fluid, double slipperiness, boolean slowDown, boolean sprint) {
        double forward = (input.forward() ? 1 : 0) - (input.backward() ? 1 : 0);
        double strafe = (input.left() ? 1 : 0) - (input.right() ? 1 : 0);
        double length = Math.hypot(forward, strafe);
        if (length == 0) return;

        forward /= length;
        strafe /= length;
        // Since 1.21.5, scaled input expands to the unit square, capped at unit length.
        double distanceToUnitSquare = 1 / Math.max(Math.abs(forward), Math.abs(strafe));
        double keyScale = Math.min(0.98 * (slowDown ? parameters.sneakingSpeed() : 1) * distanceToUnitSquare, 1);
        forward *= keyScale;
        strafe *= keyScale;
        double acceleration = fluid ? (double) 0.02f : onGround
            ? parameters.movementSpeed(sprint) * 0.21600002
                / (slipperiness * slipperiness * slipperiness)
            : (sprint ? 0.026 : 0.02);
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
    private boolean tryStepUp(
            World world, boolean onGround, double halfWidth, double height,
            Vector3d position,
            double candidateX,
            double candidateZ,
            double tickStartY) {
        if (!onGround) return false;

        java.util.OptionalDouble support = world.findHighestCollisionTop(
            candidateX - PLAYER_HALF_WIDTH, position.y + STEP_EPSILON, candidateZ - PLAYER_HALF_WIDTH,
            candidateX + PLAYER_HALF_WIDTH, tickStartY + parameters.stepHeight(), candidateZ + PLAYER_HALF_WIDTH
        );
        if (support.isEmpty()) return false;

        Vector3d stepped = new Vector3d(candidateX, support.getAsDouble(), candidateZ);
        if (isBoxColliding(world, stepped, halfWidth, height)) return false;
        position.set(stepped);
        return true;
    }

}
