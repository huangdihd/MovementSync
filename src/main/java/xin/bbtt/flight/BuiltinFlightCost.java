package xin.bbtt.flight;

import xin.bbtt.physics.PublicFlightBridge;

/** Standard glide costs; plugin modes receive no implicit glide-control penalty. */
public enum BuiltinFlightCost implements FlightCostTerm {
    APPROACH, CONTROLS;
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
