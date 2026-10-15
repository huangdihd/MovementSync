package xin.bbtt.physics;

/** Immutable, protocol-free movement inputs shared by simulation and planning. */
public record MovementParameters(
        double movementSpeed,
        double sprintingMovementSpeed,
        double jumpStrength,
        double jumpBoostVelocity,
        double gravity,
        double stepHeight,
        double sneakingSpeed,
        boolean slowFalling,
        int levitationAmplifier,
        boolean canSprint,
        double safeFallDistance,
        double fallDamageMultiplier) {
    public MovementParameters(double movementSpeed, double sprintingMovementSpeed, double jumpStrength,
            double jumpBoostVelocity, double gravity, double stepHeight, double sneakingSpeed,
            boolean slowFalling, int levitationAmplifier, boolean canSprint) {
        this(movementSpeed, sprintingMovementSpeed, jumpStrength, jumpBoostVelocity, gravity, stepHeight,
                sneakingSpeed, slowFalling, levitationAmplifier, canSprint, 3, 1);
    }

    public static final MovementParameters DEFAULT = new MovementParameters(
            (double) 0.1F, (double) 0.1F * (1.0 + (double) 0.3F),
            (double) 0.42F, 0, 0.08, 0.6, 0.3, false, -1, true);

    /** Sprint's named modifier is replaced, never compounded with the server copy. */
    public double movementSpeed(boolean sprinting) {
        return sprinting && canSprint ? sprintingMovementSpeed : movementSpeed;
    }

    public double jumpVelocity(double blockJumpMultiplier) {
        return (float) jumpStrength * (float) blockJumpMultiplier + (float) jumpBoostVelocity;
    }

    public double effectiveGravity(double verticalVelocity) {
        return slowFalling && verticalVelocity <= 0 ? Math.min(gravity, 0.01) : gravity;
    }

    /** The caller applies vertical drag afterwards; levitation replaces gravity. */
    public double verticalVelocityAfterForces(double verticalVelocity) {
        if (levitationAmplifier >= 0) {
            return verticalVelocity + (0.05 * (levitationAmplifier + 1.0) - verticalVelocity) * 0.2;
        }
        return verticalVelocity - effectiveGravity(verticalVelocity);
    }
}
