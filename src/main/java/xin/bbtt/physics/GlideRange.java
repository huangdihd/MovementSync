package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.world.World;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Obstacle-free range estimates using exactly the execution stepper. */
final class GlideRange {
    // Engineering budget: at most five minutes per pitch; exhaustion is not reachability.
    private static final int TICKS = 6000;
    private final PhysicsState initial;
    private final MovementParameters parameters;
    private final float[] pitches;
    private final Map<TrajectoryKey, Trajectory> trajectories = new HashMap<>();

    private record TrajectoryKey(float yaw, float pitch, long boostTicks) {}

    GlideRange(PhysicsState initial, MovementParameters parameters, float[] pitches) {
        this.initial = initial;
        this.parameters = parameters;
        this.pitches = pitches.clone();
    }

    /** Count-only callers assume the ordinary flight=1 component for each rocket. */
    double atHeight(double height, int fireworks, boolean attached) {
        return atHeight(height, Math.max(0L, fireworks) * minimumBoostTicks(1) + (attached ? 1 : 0));
    }

    /** Component values, one per rocket, fired consecutively without overlap. */
    double atHeight(double height, int[] rocketFlightDurations, boolean attached) {
        // An attached rocket's remaining age is unknown: credit only its next tick.
        long boostTicks = attached ? 1 : 0;
        for (int flight : rocketFlightDurations) boostTicks += minimumBoostTicks(flight);
        return atHeight(height, boostTicks);
    }

    private static long minimumBoostTicks(int flight) {
        // Yarn 1.21.11 FireworkRocketEntity: lifeTime = 10*(1+flight)+nextInt(6)+nextInt(7).
        // Use both random minima. Boost precedes ++life > lifeTime, including the expiry tick.
        return 10L * (1 + Math.max(0L, flight)) + 1;
    }

    private double atHeight(double height, long boostTicks) {
        double best = 0;
        float yaw = (float) Math.toDegrees(Math.atan2(-initial.velocity().x, initial.velocity().z));
        for (float pitch : pitches) {
            Vector3d crossing = crossing(height, yaw, pitch, boostTicks);
            if (crossing != null) {
                best = Math.max(best, Math.hypot(crossing.x - initial.position().x,
                        crossing.z - initial.position().z));
            }
        }
        return best;
    }

    /** Unpowered signed horizontal displacement along yaw, not radial distance; zero if no crossing. */
    double projectedRange(double height, float yaw, float pitch) {
        Vector3d crossing = crossing(height, yaw, pitch, 0);
        if (crossing == null) return 0;
        double radians = Math.toRadians(yaw);
        return -(crossing.x - initial.position().x) * Math.sin(radians)
                + (crossing.z - initial.position().z) * Math.cos(radians);
    }

    private Vector3d crossing(double height, float yaw, float pitch, long boostTicks) {
        var key = new TrajectoryKey(yaw, pitch, boostTicks);
        return trajectories.computeIfAbsent(key, Trajectory::new).crossing(height);
    }

    /** Cache only the requested prefix, sharing the original 6000-tick budget across queries. */
    private final class Trajectory {
        private final TrajectoryKey key;
        private final World empty = new World();
        private final GlidingPhysics powered;
        private final GlidingPhysics unpowered;
        private final List<PhysicsState> states = new ArrayList<>();

        private Trajectory(TrajectoryKey key) {
            this.key = key;
            powered = new GlidingPhysics(key.yaw(), key.pitch(), true,
                    parameters.gravity(), parameters.slowFalling());
            unpowered = new GlidingPhysics(key.yaw(), key.pitch(), false,
                    parameters.gravity(), parameters.slowFalling());
            states.add(initial);
        }

        private Vector3d crossing(double height) {
            // Heights need not be queried in order and a powered trajectory can climb again.
            // Always find the first qualifying descending step, including an invalid crossing
            // from below, rather than assuming the cached positions are monotonic in height.
            for (int tick = 0; tick < TICKS; tick++) {
                PhysicsState state = states.get(tick);
                if (states.size() == tick + 1) {
                    var result = (tick < key.boostTicks() ? powered : unpowered)
                            .step(empty, state, .299, .6);
                    states.add(new PhysicsState(result.position(), result.velocity(), false));
                }
                PhysicsState next = states.get(tick + 1);
                if (next.position().y <= height && next.velocity().y < 0) {
                    if (state.position().y >= height) {
                        double fraction = (state.position().y - height)
                                / (state.position().y - next.position().y);
                        return state.position().lerp(next.position(), fraction);
                    }
                    return null;
                }
            }
            return null;
        }
    }
}
