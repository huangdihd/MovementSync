package xin.bbtt.flight;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;

/**
 * Shared takeoff/landing arbitration on the navigation decision thread.
 * Previews and factories must be side-effect free; selection never initializes or ticks
 * a movement. Callers dispatch the winner under their navigation guard, acquiring the
 * MovementSync monitor only afterwards. Equal scores retain registration order.
 */
public final class FlightTransitions {
    private FlightTransitions() {}
    /** Transition requested by the movement lifecycle. */
    public enum Kind {
        /** Acquire airborne control; only proven-safe previews are accepted. */
        TAKEOFF,
        /** Reach support; emergency requests may minimize unavoidable damage. */
        LANDING
    }
    /**
     * Validated transition, not yet initialized; all components are non-null.
     * @param mode owning flight mode
     * @param movement caller-owned executable transition
     * @param preview trajectory evaluated by shared safety and cost rules
     */
    public record Selection(FlightMode mode, Movement movement, Trajectory preview) {}
    private record Candidate(FlightMode mode, Trajectory preview, FlightSelector.Safety safety, double cost) {}

    /**
     * Queries all available modes before constructing transitions in ranked order.
     * Takeoff always requires SAFE; landing follows the request's damage-first policy.
     * Costs are additive, ties stable. A null preview declines; a null factory result
     * tries the next validated owner. Returns null when none accepts, never permission
     * to disable airborne physics. No live state is changed by this method.
     */
    public static Selection select(FlightConfiguration configuration, FlightContext context, Kind kind) {
        var selector = new FlightSelector(configuration);
        var candidates = new ArrayList<Candidate>();
        boolean damageFirst = context.emergency() || configuration.cruiseDamageFirst();
        for (FlightMode mode : configuration.modes()) {
            if (!mode.isAvailable(context)) continue;
            Trajectory preview = kind == Kind.TAKEOFF ? mode.previewTakeoff(context) : mode.previewLanding(context);
            if (preview == null || preview.action() == null) continue;
            // Adapt recovery only: prediction, damage and terrain validation remain shared with cruise.
            FlightMode recovery = new FlightMode() {
                @Override public List<? extends FlightAction> candidates(FlightContext c) { return List.of(); }
                @Override public Trajectory predict(FlightContext c, FlightAction a) { return preview; }
                @Override public List<Trajectory> recover(FlightContext c, FlightAction a, Trajectory main) {
                    return kind == Kind.TAKEOFF ? mode.recoverTakeoff(c, main) : mode.recoverLanding(c, main);
                }
                @Override public void execute(FlightContext c, FlightAction a, MovementSync sync) {}
            };
            var safety = selector.safety(context, recovery, preview.action(), preview);
            // Starting a new flight never authorizes unavoidable injury, even during an
            // airborne rescue or with damage-first cruise enabled. Landing may minimize
            // unavoidable damage under the explicit emergency policy.
            if (safety.damageClass() == FlightSelector.DamageClass.UNKNOWN
                    || (kind == Kind.TAKEOFF || !damageFirst)
                    && safety.damageClass() != FlightSelector.DamageClass.SAFE) continue;
            double cost = 0;
            for (FlightCostTerm term : configuration.costs()) cost += term.cost(context, preview.action(), preview);
            if (Double.isFinite(cost)) candidates.add(new Candidate(mode, preview, safety, cost));
        }
        Comparator<Candidate> order = Comparator.comparingDouble(Candidate::cost);
        if (damageFirst) order = Comparator.comparing(Candidate::safety).thenComparing(order);
        candidates.sort(order);
        for (Candidate candidate : candidates) {
            Movement movement = kind == Kind.TAKEOFF ? candidate.mode().createTakeoff(context)
                    : candidate.mode().createLanding(context);
            if (movement != null) return new Selection(candidate.mode(), movement, candidate.preview());
        }
        return null;
    }
}
