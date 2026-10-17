package xin.bbtt.flight;

import java.util.List;
import java.util.Objects;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PhysicsResult;

/** Immutable bounded prediction including its terminating tick. Empty predictions are unknown. */
public record Trajectory(PhysicsState initial, List<PhysicsResult> results, FlightAction action) {
    public Trajectory(PhysicsState initial, List<PhysicsResult> results) { this(initial, results, null); }
    public Trajectory { Objects.requireNonNull(initial); results = List.copyOf(results); }
    public PhysicsState finalState() {
        if (results.isEmpty()) return initial;
        var r = results.get(results.size() - 1);
        return new PhysicsState(r.position(), r.velocity(), r.onGround(), r.fallDistance(), r.pose());
    }
}
