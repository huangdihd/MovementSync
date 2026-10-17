package xin.bbtt.flight;

import java.util.List;

/** Immutable registration snapshot. Registered hook objects themselves must be thread-safe. */
public record FlightConfiguration(List<FlightMode> modes, List<FlightCostTerm> costs,
        List<DamageRule> damageRules, boolean cruiseDamageFirst) {
    public FlightConfiguration {
        modes = List.copyOf(modes); costs = List.copyOf(costs); damageRules = List.copyOf(damageRules);
    }
    public FlightSelector.Selection select(FlightContext context) {
        return new FlightSelector(this).select(context);
    }
    /** Integration must bypass the extension selector for this exact legacy policy. */
    public boolean isLegacy() {
        return !cruiseDamageFirst && modes.size() == 1 && modes.get(0) instanceof VanillaGlideMode
                && costs.equals(List.of(BuiltinFlightCost.APPROACH, BuiltinFlightCost.CONTROLS))
                && damageRules.equals(List.of(BuiltinDamageRule.WALL, BuiltinDamageRule.FALL));
    }
}
