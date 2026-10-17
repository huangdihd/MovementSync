package xin.bbtt.flight;

import java.util.Objects;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PublicFlightBridge;

/**
 * Evaluates available registered modes synchronously on the caller's thread.
 * Safety uses the main prediction and only its mode's recovery. Selection never
 * executes controls or applies damage; callbacks must not mutate live state.
 * Instances have no per-call mutable state and can be shared if hooks and supplied
 * world/snapshot data support concurrent reads. Callback exceptions propagate.
 */
public final class FlightSelector {
    private final FlightConfiguration configuration;
    /**
     * Retains a registration snapshot without invoking its hooks.
     * @param configuration non-null immutable registration snapshot
     * @throws NullPointerException if configuration is null
     */
    public FlightSelector(FlightConfiguration configuration) { this.configuration = Objects.requireNonNull(configuration); }
    /**
     * Selected proposal, not yet executed. Component accessors return stored
     * references without side effects; hook/action thread-safety remains their
     * implementations' responsibility. The record constructor does not validate values.
     * @param mode owning mode, non-null for selector-produced results
     * @param action winning controls, expected non-null
     * @param phase descriptive stage supplied by the mode, expected non-null
     * @param trajectory non-null main prediction for selector-produced results
     */
    public record Selection(FlightMode mode, FlightAction action, FlightPhase phase, Trajectory trajectory) {}
    /** Safety categories ordered from preferred to least preferred for ranking. */
    public enum DamageClass {
        /** Known prediction resolved without raw damage. */
        SAFE,
        /** Positive raw damage, including fully mitigated impacts. */
        DAMAGING,
        /** Positive mitigated damage at least equal to current health. */
        LETHAL,
        /** Unknown terrain, invalid prediction, or unresolved recovery. */
        UNKNOWN
    }
    /**
     * Safety assessment; component accessors have no side effects. The constructor
     * stores values without validation; use valid non-null categories when creating
     * assessments manually. Unknown assessments' zero totals do not imply safety.
     * @param damageClass classification, non-null for selector-produced assessments
     * @param raw accumulated damage before mitigation in health points
     * @param reduced accumulated damage after mitigation in health points
     */
    public record Safety(DamageClass damageClass, double raw, double reduced) implements Comparable<Safety> {
        /**
         * Orders by category, then reduced damage; raw damage is not a tie-breaker.
         * @param other non-null assessment with a non-null category
         * @return negative if preferred, zero if tied, positive otherwise
         * @throws NullPointerException if either category or the other assessment is null
         */
        @Override public int compareTo(Safety other) {
            int rank = damageClass.compareTo(other.damageClass);
            return rank == 0 ? Double.compare(reduced, other.reduced) : rank;
        }
    }
    private static final Safety UNKNOWN = new Safety(DamageClass.UNKNOWN, 0, 0);
    private static final Safety SAFE = new Safety(DamageClass.SAFE, 0, 0);

    /**
     * Predicts and ranks candidates without executing the winner. Unknown safety
     * and nonfinite total costs are rejected. Normal cruise requires safety unless
     * damage-first cruise is configured; emergency always ranks by damage first.
     * Equal rankings retain the first candidate in registration/iteration order.
     * This method does not automatically retry an unsuccessful cruise as emergency.
     * @param context non-null snapshot for this selection
     * @return non-null winning proposal
     * @throws IllegalStateException if no eligible candidate has known, resolved safety
     * @throws NullPointerException if a mode returns a null prediction or a rule returns null damage
     */
    public Selection select(FlightContext context) {
        Selection best = null;
        Safety bestSafety = null;
        double bestCost = Double.POSITIVE_INFINITY;
        boolean damageFirst = context.emergency() || configuration.cruiseDamageFirst();
        for (FlightMode mode : configuration.modes()) {
            if (!mode.isAvailable(context)) continue;
            for (FlightAction action : mode.candidates(context)) {
                Trajectory trajectory = Objects.requireNonNull(mode.predict(context, action), "prediction");
                Safety safety = safety(context, mode, action, trajectory);
                // Unknown terrain or unresolved recovery is never a valid route.
                if (safety.damageClass() == DamageClass.UNKNOWN) continue;
                // Cost-only cruise matches vanilla cruise, which drops colliding trajectories: a damaging
                // candidate never wins on cost. With none safe, the caller retries with emergency ranking.
                if (!damageFirst && safety.damageClass() != DamageClass.SAFE) continue;
                double cost = 0;
                for (FlightCostTerm term : configuration.costs()) cost += term.cost(context, action, trajectory);
                if (!Double.isFinite(cost)) continue;
                int rank = bestSafety == null ? -1 : safety.compareTo(bestSafety);
                if (best != null && (damageFirst ? rank > 0 || rank == 0 && cost >= bestCost : cost >= bestCost)) continue;
                best = new Selection(mode, action, mode.phase(context, action), trajectory);
                bestSafety = safety;
                bestCost = cost;
            }
        }
        if (best == null) throw new IllegalStateException("No known flight candidate with resolved recovery");
        return best;
    }

    /**
     * Classifies a main prediction, invoking only its owning mode's recovery when
     * the horizon remains unresolved. Runs damage/recovery hooks synchronously,
     * without executing controls. Mismatched initial states and empty/unknown
     * predictions are unknown. Null or discontinuous recovery trajectories are
     * ignored; null recovery actions inherit the main action. The best resolved
     * recovery assessment is used, or unknown if none resolves.
     * @param context non-null planning snapshot
     * @param mode non-null owner of the prediction and recovery strategy
     * @param action non-null controls used for the main prediction
     * @param main non-null prediction starting at the context state
     * @return non-null assessment, possibly {@link DamageClass#UNKNOWN}
     * @throws NullPointerException if a damage rule returns null
     */
    public Safety safety(FlightContext context, FlightMode mode, FlightAction action, Trajectory main) {
        if (!sameState(context.state(), main.initial())) return UNKNOWN;
        Safety terminal = classify(context, action, main);
        if (terminal != null) return terminal;
        Safety best = UNKNOWN;
        for (Trajectory recovery : mode.recover(context, action, main)) {
            if (recovery == null || !sameState(main.finalState(), recovery.initial())) continue;
            Safety result = classify(context, recovery.action() == null ? action : recovery.action(), recovery);
            if (result != null && result.compareTo(best) < 0) best = result;
        }
        return best;
    }

    private static boolean sameState(PhysicsState a, PhysicsState b) {
        return a.position().equals(b.position()) && a.velocity().equals(b.velocity())
                && a.onGround() == b.onGround() && Double.compare(a.fallDistance(), b.fallDistance()) == 0;
    }

    /** Null denotes an exhausted horizon still requiring mode-owned recovery. */
    private Safety classify(FlightContext context, FlightAction action, Trajectory trajectory) {
        if (trajectory.results().isEmpty()) return UNKNOWN;
        PhysicsState state = trajectory.initial();
        double raw = 0, reduced = 0;
        for (var result : trajectory.results()) {
            if (!PublicFlightBridge.known(context, state, result)
                    || !Double.isFinite(result.fallDistance()) || result.fallDistance() < 0) return UNKNOWN;
            // Do not trust endpoint-only plugin models to tunnel through solid terrain.
            // Reported impacts are classified by the shared damage rules instead.
            var from = state.position();
            var to = result.position();
            if (!result.onGround() && !result.collidedX() && !result.collidedZ()
                    && context.world().isBoxColliding(Math.min(from.x, to.x) - .299,
                    Math.min(from.y, to.y), Math.min(from.z, to.z) - .299,
                    Math.max(from.x, to.x) + .299, Math.max(from.y, to.y) + .6,
                    Math.max(from.z, to.z) + .299)) return UNKNOWN;
            for (DamageRule rule : configuration.damageRules()) {
                var damage = Objects.requireNonNull(rule.damage(context, action, state, result));
                // Environmental hazards are forbidden cells, not an impact trade-off.
                if ((rule == BuiltinDamageRule.LAVA || rule == BuiltinDamageRule.FIRE) && damage.raw() > 0)
                    return UNKNOWN;
                raw += damage.raw(); reduced += damage.reduced();
            }
            if (!Double.isFinite(raw) || !Double.isFinite(reduced)) return UNKNOWN;
            if (result.onGround() || raw > 0) return raw == 0 ? SAFE
                    : new Safety(reduced > 0 && reduced >= context.health() ? DamageClass.LETHAL : DamageClass.DAMAGING, raw, reduced);
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance(), result.pose());
        }
        return state.velocity().y > -.5 ? SAFE : null;
    }
}
