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

    public GlidingPhysics(float yaw, float pitch, boolean firework) {
        this.yaw = yaw;
        this.pitch = pitch;
        this.firework = firework;
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
        float yawRadians = -yaw * DEGREES_TO_RADIANS - (float) Math.PI;
        float pitchRadians = pitch * DEGREES_TO_RADIANS;
        float pitchBase = -cos(-pitchRadians);
        Vector3d look = new Vector3d(sin(yawRadians) * pitchBase,
                sin(-pitchRadians), cos(yawRadians) * pitchBase);
        double lookHorizontal = Math.sqrt(look.x * look.x + look.z * look.z);
        double velocityHorizontal = Math.sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
        float pitchCosine = cos(pitchRadians);
        float lift = (float) ((double) pitchCosine * pitchCosine * Math.min(1, look.length() / 0.4));

        velocity.y += -0.08 + lift * 0.06;
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

        boolean collidedX = false;
        boolean collidedZ = false;
        if (velocity.x != 0) {
            Vector3d candidate = new Vector3d(position).add(velocity.x, 0, 0);
            if (collides(world, candidate, halfWidth, height)) {
                velocity.x = 0;
                collidedX = true;
            } else {
                position.x = candidate.x;
            }
        }
        if (velocity.z != 0) {
            Vector3d candidate = new Vector3d(position).add(0, 0, velocity.z);
            if (collides(world, candidate, halfWidth, height)) {
                velocity.z = 0;
                collidedZ = true;
            } else {
                position.z = candidate.z;
            }
        }
        boolean onGround = false;
        if (velocity.y < 0) {
            double desiredY = position.y + velocity.y;
            double resolvedY = VerticalCollisionResolver.resolveDownwardY(world, position, halfWidth, velocity.y);
            // Include exact contact, not just motion clipped short of the destination.
            onGround = resolvedY > desiredY + 1.0e-9
                    || world.findHighestCollisionTop(position.x - halfWidth, desiredY - 1.0e-9,
                            position.z - halfWidth, position.x + halfWidth, desiredY + 1.0e-9,
                            position.z + halfWidth).isPresent();
            position.y = resolvedY;
            if (onGround) velocity.y = 0;
        } else if (velocity.y > 0) {
            Vector3d candidate = new Vector3d(position).add(0, velocity.y, 0);
            if (collides(world, candidate, halfWidth, height)) {
                velocity.y = 0;
            } else {
                position.y = candidate.y;
            }
        }

        // Baritone applies the attached rocket after travel: it changes next tick's
        // motion, not the displacement just calculated. Landing ends flight first.
        if (firework && !onGround) {
            velocity.add(look.x * 0.1 + (look.x * 1.5 - velocity.x) * 0.5,
                    look.y * 0.1 + (look.y * 1.5 - velocity.y) * 0.5,
                    look.z * 0.1 + (look.z * 1.5 - velocity.z) * 0.5);
        }
        return new PhysicsResult(position, velocity, collidedX, collidedZ, onGround);
    }

    private static boolean collides(World world, Vector3d position, double halfWidth, double height) {
        return world.isBoxColliding(position.x - halfWidth, position.y, position.z - halfWidth,
                position.x + halfWidth, position.y + height, position.z + halfWidth);
    }
}
