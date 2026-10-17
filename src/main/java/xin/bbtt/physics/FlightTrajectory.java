package xin.bbtt.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiPredicate;
import xin.bbtt.world.World;

/** Internal tick sequence. Includes the terminating tick and the first physical collision. */
final class FlightTrajectory {
    private final PhysicsState initial;
    private final List<PhysicsResult> results;
    private final int firstCollision;

    private FlightTrajectory(PhysicsState initial, List<PhysicsResult> results, int firstCollision) {
        this.initial = initial;
        this.results = List.copyOf(results);
        this.firstCollision = firstCollision;
    }
    PhysicsState initial() { return initial; }
    PhysicsResult result(int tick) { return results.get(tick); }
    int size() { return results.size(); }
    int firstCollision() { return firstCollision; }

    static FlightTrajectory simulate(World world, PhysicsState initial, PhysicsStepper physics,
            int horizon, BiPredicate<PhysicsState, PhysicsResult> stop) {
        var results = new ArrayList<PhysicsResult>(horizon);
        PhysicsState state = initial;
        int collision = -1;
        for (int tick = 0; tick < horizon; tick++) {
            PhysicsResult result = physics.step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            results.add(result);
            if (collision < 0 && (result.onGround() || result.collidedX() || result.collidedZ())) collision = tick;
            if (stop.test(state, result)) break;
            state = new PhysicsState(result.position(), result.velocity(), false, result.fallDistance());
        }
        return new FlightTrajectory(initial, results, collision);
    }
}
