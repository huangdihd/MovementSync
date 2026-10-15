package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.tasks.VerticalCollisionResolver;
import xin.bbtt.world.World;

/** One immutable set of flight inputs, in degrees, reusable for look-ahead steps. */
public final class GlidingPhysics implements PhysicsStepper {
    private static final float DEGREES_TO_RADIANS = (float) (Math.PI / 180.0);
    private static final float[] SINES = new float[65536];

    static {
        // Minecraft's lookup-table rounding is part of the movement calculation.
        for (int index = 0; index < SINES.length; index++) {
            SINES[index] = (float) Math.sin(index * Math.PI * 2.0 / 65536.0);
        }
    }

    private final float yaw;
    private final float pitch;
    private final boolean firework;
    private final double gravity;
    private final boolean slowFalling;

    public GlidingPhysics(float yaw, float pitch, boolean firework) {
        this(yaw, pitch, firework, 0.08, false);
    }

    public GlidingPhysics(float yaw, float pitch, boolean firework, double gravity, boolean slowFalling) {
        this.yaw = yaw;
        this.pitch = pitch;
        this.firework = firework;
        this.gravity = gravity;
        this.slowFalling = slowFalling;
    }

    private static float sin(float angle) {
        return SINES[(int) (angle * 10430.378F) & 65535];
    }

    private static float cos(float angle) {
        return SINES[(int) (angle * 10430.378F + 16384.0F) & 65535];
    }

    @Override
    public PhysicsResult step(World world, PhysicsState state, double halfWidth, double height) {
        Vector3d position = state.position();
        Vector3d velocity = state.velocity();
        // LivingEntity.tickMovement snaps player motion before travel.
        if (velocity.x * velocity.x + velocity.z * velocity.z < 9.0E-6) {
            velocity.x = 0;
            velocity.z = 0;
        }
        if (Math.abs(velocity.y) < 0.003) velocity.y = 0;
        float yawRadians = -yaw * DEGREES_TO_RADIANS;
        float pitchRadians = pitch * DEGREES_TO_RADIANS;
        float pitchCos = cos(pitchRadians);
        Vector3d look = new Vector3d(sin(yawRadians) * pitchCos,
                -sin(pitchRadians), cos(yawRadians) * pitchCos);
        double lookHorizontal = Math.sqrt(look.x * look.x + look.z * look.z);
        double velocityHorizontal = Math.sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
        // Since 1.18.2, lift uses double-precision Java cosine, not the
        // lookup table still used by the look vector and pitch acceleration.
        double pitchCosine = Math.cos(pitchRadians);
        double lift = pitchCosine * pitchCosine;

        // LivingEntity.calcGlidingVelocity uses getEffectiveGravity, including slow falling.
        double effectiveGravity = slowFalling && velocity.y <= 0 ? Math.min(gravity, 0.01) : gravity;
        velocity.y += effectiveGravity * (-1.0 + lift * 0.75);
        if (velocity.y < 0 && lookHorizontal > 0) {
            double amount = velocity.y * -0.1 * lift;
            velocity.add(look.x * amount / lookHorizontal, amount, look.z * amount / lookHorizontal);
        }
        if (pitchRadians < 0 && lookHorizontal > 0) {
            double amount = velocityHorizontal * (double) (-sin(pitchRadians)) * 0.04;
            velocity.add(-look.x * amount / lookHorizontal, amount * 3.2,
                    -look.z * amount / lookHorizontal);
        }
        if (lookHorizontal > 0) {
            velocity.x += (look.x / lookHorizontal * velocityHorizontal - velocity.x) * 0.1;
            velocity.z += (look.z / lookHorizontal * velocityHorizontal - velocity.z) * 0.1;
        }
        velocity.mul((double) 0.99f, (double) 0.98f, (double) 0.99f);

        double fallDistance = slowFalling ? 0 : state.fallDistance();
        // Gliding limits accumulated fall damage while descending slowly.
        if (state.velocity().y > -0.5 && fallDistance > 1) fallDistance = 1;
        // Entity.move clips Y first, then the larger horizontal component. In
        // particular, falling below a platform must happen before testing its side.
        boolean onGround = false;
        if (velocity.y < 0) {
            double desiredY = position.y + velocity.y;
            double resolvedY = VerticalCollisionResolver.resolveDownwardY(world, position, halfWidth, velocity.y, fallDistance);
            onGround = resolvedY > desiredY + 1.0e-9
                    || world.findHighestCollisionTop(position.x - halfWidth, desiredY - 1.0e-9,
                            position.z - halfWidth, position.x + halfWidth, desiredY + 1.0e-9,
                            position.z + halfWidth).isPresent();
            position.y = resolvedY;
            if (onGround) velocity.y = 0;
        } else if (velocity.y > 0) {
            moveAxis(world, position, velocity, halfWidth, height, 1);
        }
        boolean zFirst = Math.abs(velocity.x) < Math.abs(velocity.z);
        boolean collidedX;
        boolean collidedZ;
        if (zFirst) {
            collidedZ = moveAxis(world, position, velocity, halfWidth, height, 2);
            collidedX = moveAxis(world, position, velocity, halfWidth, height, 0);
        } else {
            collidedX = moveAxis(world, position, velocity, halfWidth, height, 0);
            collidedZ = moveAxis(world, position, velocity, halfWidth, height, 2);
        }

        // FireworkRocketEntity ticks after player travel. Contact does not clear
        // the client's gliding flag, so an attached rocket still boosts on ground.
        if (firework) {
            velocity.add(look.x * 0.1 + (look.x * 1.5 - velocity.x) * 0.5,
                    look.y * 0.1 + (look.y * 1.5 - velocity.y) * 0.5,
                    look.z * 0.1 + (look.z * 1.5 - velocity.z) * 0.5);
        }
        fallDistance = onGround ? 0 : fallDistance + Math.max(0, state.position().y - position.y);
        return new PhysicsResult(position, velocity, collidedX, collidedZ, onGround, fallDistance);
    }

    private static boolean moveAxis(World world, Vector3d position, Vector3d velocity,
            double halfWidth, double height, int axis) {
        double requested = velocity.get(axis);
        // Sweep the entire displacement, not just its endpoint: a fast glide
        // can cross a thin collision shape and end in clear air.
        double clipped = WalkingPhysics.clipMovement(world, position, halfWidth, height, axis, requested);
        position.setComponent(axis, position.get(axis) + clipped);
        boolean collided = clipped != requested;
        if (collided) velocity.setComponent(axis, 0);
        return collided;
    }
}
