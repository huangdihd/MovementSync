package xin.bbtt.flight;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Fluent registration builder. Instances are mutable and not thread-safe; confine
 * each to one thread. Static global registration and its build-time snapshot are
 * synchronized. Hook objects are shared, not cloned, and must be thread-safe.
 * Global hooks are included only after {@link #addStandard()}, at build time;
 * subsequent registration changes cannot alter an existing navigation snapshot.
 * The implicit constructor creates an empty builder with cost-only safe cruise.
 */
public final class FlightContextBuilder {
    private static final List<FlightMode> GLOBAL_MODES = new ArrayList<>();
    private static final List<FlightCostTerm> GLOBAL_COSTS = new ArrayList<>();
    private static final List<DamageRule> GLOBAL_DAMAGE = new ArrayList<>();
    private final List<FlightMode> modes = new ArrayList<>();
    private final List<FlightCostTerm> costs = new ArrayList<>();
    private final List<DamageRule> damage = new ArrayList<>();
    private boolean cruiseDamageFirst;
    private boolean standard;
    /**
     * Appends a global mode for future standard builds; existing snapshots are unchanged.
     * Registration is synchronized, preserves duplicates, and does not invoke the hook.
     * @param mode non-null thread-safe mode
     * @throws NullPointerException if the hook is null
     */
    public static synchronized void registerGlobalMode(FlightMode mode) { GLOBAL_MODES.add(Objects.requireNonNull(mode)); }
    /**
     * Appends a global cost term for future standard builds; existing snapshots are unchanged.
     * Registration is synchronized, preserves duplicates, and does not invoke the hook.
     * @param cost non-null thread-safe cost term
     * @throws NullPointerException if the hook is null
     */
    public static synchronized void registerGlobalCostTerm(FlightCostTerm cost) { GLOBAL_COSTS.add(Objects.requireNonNull(cost)); }
    /**
     * Appends a global damage rule for future standard builds; existing snapshots are unchanged.
     * Registration is synchronized, preserves duplicates, and does not invoke the hook.
     * @param rule non-null thread-safe damage rule
     * @throws NullPointerException if the hook is null
     */
    public static synchronized void registerGlobalDamageRule(DamageRule rule) { GLOBAL_DAMAGE.add(Objects.requireNonNull(rule)); }
    /**
     * Appends a local mode in evaluation order, retaining duplicates without invoking it.
     * @param mode non-null thread-safe mode
     * @return this mutable builder
     * @throws NullPointerException if the hook is null
     */
    public FlightContextBuilder addMode(FlightMode mode) { modes.add(Objects.requireNonNull(mode)); return this; }
    /**
     * Appends a local cost term in evaluation order, retaining duplicates without invoking it.
     * @param cost non-null thread-safe cost term
     * @return this mutable builder
     * @throws NullPointerException if the hook is null
     */
    public FlightContextBuilder addCostTerm(FlightCostTerm cost) { costs.add(Objects.requireNonNull(cost)); return this; }
    /**
     * Appends a local damage rule in evaluation order, retaining duplicates without invoking it.
     * @param rule non-null thread-safe damage rule
     * @return this mutable builder
     * @throws NullPointerException if the hook is null
     */
    public FlightContextBuilder addDamageRule(DamageRule rule) { damage.add(Objects.requireNonNull(rule)); return this; }
    /**
     * Off (default): cruise only accepts candidates proven safe and ranks them by cost; when none is safe the
     * movement switches to emergency ranking. On: cruise itself ranks damaging candidates by damage, then cost,
     * so it may keep cruising through a damaging manoeuvre instead of entering an emergency landing.
     * Changes only this builder; emergency selection always ranks damage first.
     * @param enabled whether to rank cruise damage before cost
     * @return this mutable builder
     */
    public FlightContextBuilder cruiseDamageFirst(boolean enabled) { cruiseDamageFirst = enabled; return this; }
    /**
     * Appends standard glide, approach/control costs, and wall/fall/lava/fire rules once.
     * Also enables inclusion of global hooks at each subsequent build, after local
     * hooks. Repeated calls do not append another standard set or execute hooks.
     * @return this mutable builder
     */
    public FlightContextBuilder addStandard() {
        if (!standard) {
            standard = true;
            addMode(VanillaGlideMode.INSTANCE);
            addCostTerm(BuiltinFlightCost.APPROACH).addCostTerm(BuiltinFlightCost.CONTROLS);
            addDamageRule(BuiltinDamageRule.WALL).addDamageRule(BuiltinDamageRule.FALL)
                    .addDamageRule(BuiltinDamageRule.LAVA).addDamageRule(BuiltinDamageRule.FIRE);
        }
        return this;
    }
    /**
     * Copies local registrations and, when enabled, the current global registrations.
     * Does not clear this builder or invoke hooks. The global copy is synchronized;
     * concurrent mutation of this builder is not supported.
     * @return non-null independent list snapshot sharing the registered hook objects
     */
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
