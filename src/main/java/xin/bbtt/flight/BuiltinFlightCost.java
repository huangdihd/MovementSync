package xin.bbtt.flight;

import xin.bbtt.physics.PublicFlightBridge;

/**
 * Stateless, thread-safe standard costs, evaluated synchronously during selection.
 * These read the planning snapshot and world without executing player controls.
 * Plugin actions receive destination-distance cost but no glide-control penalty.
 */
public enum BuiltinFlightCost implements FlightCostTerm {
    /** Route/landing-aware glide approach cost, or endpoint distance times three for other actions. */
    APPROACH,
    /** Glide attitude/boost penalty; zero for actions other than {@link FlightAction.Glide}. */
    CONTROLS;
    /**
     * Computes this term without executing the action.
     * @param c non-null planning snapshot
     * @param action non-null proposed controls
     * @param t non-null main prediction
     * @return additive cost; approach returns positive infinity for an empty prediction
     */
    @Override public double cost(FlightContext c, FlightAction action, Trajectory t) {
        if (!(action instanceof FlightAction.Glide a)) {
            if (this == CONTROLS) return 0;
            if (t.results().isEmpty()) return Double.POSITIVE_INFINITY;
            // All modes share destination progress; glide-specific attitude costs do not apply.
            return t.finalState().position().distance(c.target()) * 3;
        }
        return this == APPROACH ? PublicFlightBridge.approach(c, a, t) : PublicFlightBridge.controls(c, a);
    }
}
