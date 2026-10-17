package xin.bbtt.flight;

/**
 * Extensible manoeuvre stage; plugins may supply their own enum.
 * Implementations should be immutable and thread-safe: phases describe selection
 * results and do not themselves execute movement or change player state.
 */
public interface FlightPhase {
    /**
     * Returns a stable diagnostic name, synchronously and without side effects.
     * @return non-null phase name
     */
    String name();
    /** Standard descriptive stages; selection does not enforce transitions between them. */
    enum Standard implements FlightPhase {
        /** Gaining altitude. */
        CLIMB,
        /** Travelling toward the destination. */
        CRUISE,
        /** Losing altitude on approach. */
        DESCENT,
        /** Steering around an obstacle. */
        AVOIDANCE,
        /** Making a planned landing. */
        LANDING,
        /** Making an emergency landing. */
        FORCED_LANDING,
        /** Reached the ground. */
        LANDED
    }
}
