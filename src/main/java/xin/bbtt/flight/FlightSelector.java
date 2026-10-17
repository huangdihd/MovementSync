package xin.bbtt.flight;

import java.util.Objects;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PublicFlightBridge;

/** Evaluates every registered mode. Safety uses the main prediction and only that mode's recovery. */
public final class FlightSelector {
    private final FlightConfiguration configuration;
    public FlightSelector(FlightConfiguration configuration) { this.configuration = Objects.requireNonNull(configuration); }
    public record Selection(FlightMode mode, FlightAction action, FlightPhase phase, Trajectory trajectory) {}
    public enum DamageClass { SAFE, DAMAGING, LETHAL, UNKNOWN }
    public record Safety(DamageClass damageClass, double raw, double reduced) implements Comparable<Safety> {
        @Override public int compareTo(Safety other) {
            int rank = damageClass.compareTo(other.damageClass);
            return rank == 0 ? Double.compare(reduced, other.reduced) : rank;
        }
    }
    private static final Safety UNKNOWN = new Safety(DamageClass.UNKNOWN, 0, 0);
    private static final Safety SAFE = new Safety(DamageClass.SAFE, 0, 0);

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
