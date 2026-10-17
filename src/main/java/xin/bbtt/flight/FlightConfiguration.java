package xin.bbtt.flight;

import java.util.List;

/**
 * Immutable registration snapshot for navigation. Lists are copied, but registered
 * hook objects are shared and must themselves be thread-safe. Component accessors
 * expose unmodifiable lists and the stored policy without side effects; later
 * builder/global registration changes cannot alter this snapshot.
 *
 * @param modes non-null modes in candidate evaluation order, with no null elements
 * @param costs non-null additive costs in evaluation order, with no null elements
 * @param damageRules non-null additive per-tick rules, with no null elements
 * @param cruiseDamageFirst whether cruise ranks by damage before cost rather than requiring safety
 */
public record FlightConfiguration(List<FlightMode> modes, List<FlightCostTerm> costs,
        List<DamageRule> damageRules, boolean cruiseDamageFirst) {
    /**
     * Copies registration lists; does not run any registered hook.
     * @param modes modes to consider, in order
     * @param costs additive ranking terms
     * @param damageRules additive tick damage rules
     * @param cruiseDamageFirst whether cruise uses damage-first ranking
     * @throws NullPointerException if any list or list element is null
     */
    public FlightConfiguration {
        modes = List.copyOf(modes); costs = List.copyOf(costs); damageRules = List.copyOf(damageRules);
    }
    /**
     * Selects synchronously on the caller's thread without executing controls.
     * Hooks may be invoked repeatedly; failures propagate to the caller.
     * @param context non-null snapshot for this selection
     * @return non-null winning candidate
     * @throws IllegalStateException if no known eligible candidate has resolved recovery
     */
    public FlightSelector.Selection select(FlightContext context) {
        return new FlightSelector(this).select(context);
    }
    /**
     * Tests for the exact legacy policy without invoking hooks. Integration must
     * bypass the extension selector for this policy to preserve legacy behavior.
     * @return whether only the standard glide mode, costs and damage rules are configured
     */
    public boolean isLegacy() {
        return !cruiseDamageFirst && modes.size() == 1 && modes.get(0) instanceof VanillaGlideMode
                && costs.equals(List.of(BuiltinFlightCost.APPROACH, BuiltinFlightCost.CONTROLS))
                && damageRules.equals(List.of(BuiltinDamageRule.WALL, BuiltinDamageRule.FALL));
    }
}
