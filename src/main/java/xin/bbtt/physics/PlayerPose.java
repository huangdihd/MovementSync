package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Local tick-end pose, distinct from the asynchronously received gliding flag. */
public enum PlayerPose {
    STANDING(1.8), CROUCHING(1.5), SWIMMING(0.6), GLIDING(0.6);

    private final double height;
    PlayerPose(double height) { this.height = height; }
    public double height() { return height; }

    public boolean shouldSlowDown(boolean gliding, boolean touchingWater, boolean sneak) {
        return (sneak || this == CROUCHING) && !touchingWater
                || !touchingWater && (this == SWIMMING || this == GLIDING && !gliding);
    }

    /** PlayerEntity.updatePose runs after travel, retaining pose if even crawling cannot fit. */
    public PlayerPose afterTravel(World world, Vector3d position, boolean gliding, boolean swimming, boolean sneak) {
        if (!fits(world, position, SWIMMING)) return this;
        PlayerPose expected = swimming ? SWIMMING : gliding ? GLIDING : sneak ? CROUCHING : STANDING;
        return fits(world, position, expected) ? expected
                : fits(world, position, CROUCHING) ? CROUCHING : SWIMMING;
    }

    private static boolean fits(World world, Vector3d p, PlayerPose pose) {
        double radius = (double) 0.6f / 2 - 1.0e-7;
        return !world.isBoxColliding(p.x - radius, p.y + 1.0e-7, p.z - radius,
                p.x + radius, p.y + (double)(float)pose.height - 1.0e-7, p.z + radius);
    }
}
