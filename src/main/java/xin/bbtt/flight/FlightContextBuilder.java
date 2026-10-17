package xin.bbtt.flight;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Global registrations are copied at build time; subsequent changes cannot alter a navigation snapshot. */
public final class FlightContextBuilder {
    private static final List<FlightMode> GLOBAL_MODES = new ArrayList<>();
    private static final List<FlightCostTerm> GLOBAL_COSTS = new ArrayList<>();
    private static final List<DamageRule> GLOBAL_DAMAGE = new ArrayList<>();
    private final List<FlightMode> modes = new ArrayList<>();
    private final List<FlightCostTerm> costs = new ArrayList<>();
    private final List<DamageRule> damage = new ArrayList<>();
    private boolean cruiseDamageFirst;
    private boolean standard;
    public static synchronized void registerGlobalMode(FlightMode mode) { GLOBAL_MODES.add(Objects.requireNonNull(mode)); }
    public static synchronized void registerGlobalCostTerm(FlightCostTerm cost) { GLOBAL_COSTS.add(Objects.requireNonNull(cost)); }
    public static synchronized void registerGlobalDamageRule(DamageRule rule) { GLOBAL_DAMAGE.add(Objects.requireNonNull(rule)); }
    public FlightContextBuilder addMode(FlightMode mode) { modes.add(Objects.requireNonNull(mode)); return this; }
    public FlightContextBuilder addCostTerm(FlightCostTerm cost) { costs.add(Objects.requireNonNull(cost)); return this; }
    public FlightContextBuilder addDamageRule(DamageRule rule) { damage.add(Objects.requireNonNull(rule)); return this; }
    /**
     * Off (default): cruise only accepts candidates proven safe and ranks them by cost; when none is safe the
     * movement switches to emergency ranking. On: cruise itself ranks damaging candidates by damage, then cost,
     * so it may keep cruising through a damaging manoeuvre instead of entering an emergency landing.
     */
    public FlightContextBuilder cruiseDamageFirst(boolean enabled) { cruiseDamageFirst = enabled; return this; }
    public FlightContextBuilder addStandard() {
        if (!standard) {
            standard = true;
            addMode(VanillaGlideMode.INSTANCE);
            addCostTerm(BuiltinFlightCost.APPROACH).addCostTerm(BuiltinFlightCost.CONTROLS);
            addDamageRule(BuiltinDamageRule.WALL).addDamageRule(BuiltinDamageRule.FALL);
        }
        return this;
    }
    public FlightConfiguration build() {
        synchronized (FlightContextBuilder.class) {
            var m = new ArrayList<>(modes);
            var c = new ArrayList<>(costs);
            var d = new ArrayList<>(damage);
            if (standard) {
                m.addAll(GLOBAL_MODES);
                c.addAll(GLOBAL_COSTS);
                d.addAll(GLOBAL_DAMAGE);
            }
            return new FlightConfiguration(m, c, d, cruiseDamageFirst);
        }
    }
}
