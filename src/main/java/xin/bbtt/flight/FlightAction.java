package xin.bbtt.flight;

/**
 * Mode-owned control proposal, not an instruction executed on construction.
 * Actions can be shared between planning callbacks and later tick execution;
 * plugin implementations should be immutable and thread-safe.
 */
public interface FlightAction {
    /**
     * Vanilla glide controls. Component accessors have no side effects; requesting
     * a firework does not consume one until the mode executes the action.
     * @param yaw finite heading in degrees
     * @param pitch finite pitch in degrees (positive looks down)
     * @param firework whether execution should request a firework boost
     */
    record Glide(float yaw, float pitch, boolean firework) implements FlightAction {
        /**
         * Validates controls without normalizing or clamping angles.
         * @param yaw heading in degrees
         * @param pitch pitch in degrees
         * @param firework whether to request a boost
         * @throws IllegalArgumentException if either angle is nonfinite
         */
        public Glide {
            if (!Float.isFinite(yaw) || !Float.isFinite(pitch)) throw new IllegalArgumentException("Nonfinite controls");
        }
    }
}
