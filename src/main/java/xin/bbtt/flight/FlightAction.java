package xin.bbtt.flight;

/** Immutable mode-owned control proposal. Plugin actions should be immutable too. */
public interface FlightAction {
    record Glide(float yaw, float pitch, boolean firework) implements FlightAction {
        public Glide {
            if (!Float.isFinite(yaw) || !Float.isFinite(pitch)) throw new IllegalArgumentException("Nonfinite controls");
        }
    }
}
