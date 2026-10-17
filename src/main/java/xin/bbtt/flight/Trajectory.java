package xin.bbtt.flight;

import java.util.List;
import java.util.Objects;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PhysicsResult;

/**
 * Bounded prediction including its terminating tick, with no live-player effects.
 * The result list is copied and exposed unmodifiable; contained physics values
 * are not deep-copied and must be treated as read-only when shared across threads.
 * Empty predictions are unknown to the selector. Component accessors return the
 * stored values without advancing simulation.
 *
 * @param initial non-null state immediately before the first predicted tick
 * @param results non-null ordered tick results with no null elements
 * @param action recovery-specific controls, or null to inherit the main action
 */
public record Trajectory(PhysicsState initial, List<PhysicsResult> results, FlightAction action) {
    /**
     * Creates a prediction whose controls are inherited from the main action.
     * @param initial non-null starting state
     * @param results non-null ordered results with no null elements
     * @throws NullPointerException if the state, list, or a list element is null
     */
    public Trajectory(PhysicsState initial, List<PhysicsResult> results) { this(initial, results, null); }
    /**
     * Copies the result list without running physics.
     * @param initial non-null starting state
     * @param results non-null ordered results with no null elements
     * @param action recovery controls, or null to inherit the main action
     * @throws NullPointerException if the state, list, or a list element is null
     */
    public Trajectory { Objects.requireNonNull(initial); results = List.copyOf(results); }
    /**
     * Reads the prediction endpoint without simulating another tick.
     * @return initial state for an empty prediction, otherwise a state from the last result
     */
    public PhysicsState finalState() {
        if (results.isEmpty()) return initial;
        var r = results.get(results.size() - 1);
        return new PhysicsState(r.position(), r.velocity(), r.onGround(), r.fallDistance(), r.pose());
    }
}
