package xin.bbtt.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Bounded local steering only: no route search, rocket requests, or terminal safety proof. */
public final class LocalLookaheadSolver {
    public enum Status { COMPLETE, NO_SAFE_CANDIDATE, BUDGET_EXHAUSTED }

    /** All candidates share this budget. A zero time limit means step-count-only. */
    public record Budget(int maxSteps, int horizon, long maxNanos) {
        public Budget {
            if (maxSteps < 0 || horizon < 1 || maxNanos < 0) throw new IllegalArgumentException("Invalid budget");
        }
    }

    /** Null decision means that no full-horizon safe candidate was evaluated. */
    public record Result(ElytraFlightController.Decision decision, int steps, long elapsedNanos,
            Status status, PhysicsState predictedState, double score) {}

    /** Immutable constant-velocity observation; prediction time is measured in game ticks. */
    public record MovingTarget(Vector3d position, Vector3d velocity) {
        public MovingTarget {
            position = checked(position);
            velocity = checked(velocity);
        }
        @Override public Vector3d position() { return new Vector3d(position); }
        @Override public Vector3d velocity() { return new Vector3d(velocity); }
        public Vector3d predict(double ticks) {
            if (!Double.isFinite(ticks) || ticks < 0) throw new IllegalArgumentException("Invalid prediction time");
            return new Vector3d(velocity).mul(ticks).add(position);
        }
        /** Higher is better. This seam is intentionally not connected to entity follow. */
        public double score(PhysicsState state, double ticks) {
            return -state.position().distanceSquared(predict(ticks))
                    - .25 * state.velocity().distanceSquared(velocity);
        }
    }

    public Result solve(List<Vector3d> lookahead, PhysicsState initial, World world,
            MovementParameters movement, float yaw, float pitch, boolean attached, Budget budget) {
        return solve(lookahead, initial, world, movement, yaw, pitch, attached, budget, null);
    }

    /** Route points are copied and capped at three yaw anchors; movingTarget is an optional scoring seam. */
    public Result solve(List<Vector3d> lookahead, PhysicsState initial, World world,
            MovementParameters movement, float yaw, float pitch, boolean attached, Budget budget,
            MovingTarget movingTarget) {
        Objects.requireNonNull(world);
        Objects.requireNonNull(movement);
        Objects.requireNonNull(budget);
        if (!Float.isFinite(yaw) || !Float.isFinite(pitch)) throw new IllegalArgumentException("Invalid angles");
        checked(initial.position());
        checked(initial.velocity());
        if (lookahead.isEmpty()) throw new IllegalArgumentException("Empty lookahead");
        List<Vector3d> route = lookahead.stream().map(LocalLookaheadSolver::checked)
                .sorted(java.util.Comparator.comparingDouble(p -> p.distanceSquared(initial.position()))).toList();
        long started = System.nanoTime();
        List<Float> yaws = new ArrayList<>();
        for (int i = 0; i < Math.min(3, route.size()); i++) {
            Vector3d delta = new Vector3d(route.get(i)).sub(initial.position());
            addYaw(yaws, delta.x == 0 && delta.z == 0 ? yaw
                    : (float) Math.toDegrees(Math.atan2(-delta.x, delta.z)));
        }
        addYaw(yaws, yaw);
        addYaw(yaws, yaw - 30);
        addYaw(yaws, yaw + 30);
        float center = Math.max(-75, Math.min(60, pitch));
        float[] pitches = {center, Math.max(-80, center - 15), Math.min(70, center + 15), -45, 20};
        int steps = 0;
        ElytraFlightController.Decision best = null;
        PhysicsState bestState = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        boolean exhausted = false;
        candidates: for (float candidateYaw : yaws) {
            for (float candidatePitch : pitches) {
                PhysicsState state = new PhysicsState(initial.position(), initial.velocity(), initial.onGround(),
                        initial.fallDistance(), PlayerPose.GLIDING);
                GlidingPhysics physics = new GlidingPhysics(candidateYaw, candidatePitch, attached,
                        PlayerInput.NONE, movement);
                for (int tick = 0; tick < budget.horizon(); tick++) {
                    if (steps >= budget.maxSteps() || budget.maxNanos() > 0
                            && System.nanoTime() - started >= budget.maxNanos()) {
                        exhausted = true;
                        break candidates;
                    }
                    PhysicsResult result = physics.step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                    steps++;
                    // GlidingPhysics has no collidedY flag: an upward velocity clipped to zero is a ceiling hit.
                    if (!FlightSafety.knownStep(world, state, result) || FlightSafety.collision(result)
                            || touchesCollision(world, result.position())
                            || state.velocity().y > .003 && result.velocity().y == 0
                            || FlightHazards.damage(world, state, gliding(result), DamageParameters.NONE, true).raw() > 0
                            || FlightHazards.damage(world, state, gliding(result), DamageParameters.NONE, false).raw() > 0) {
                        state = null;
                        break;
                    }
                    state = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                            result.fallDistance(), PlayerPose.GLIDING);
                }
                if (state == null) continue;
                double score = routeScore(route, initial.position(), state.position());
                if (movingTarget != null) score += movingTarget.score(state, budget.horizon());
                if (score > bestScore) {
                    bestScore = score;
                    bestState = state;
                    best = new ElytraFlightController.Decision(candidateYaw, candidatePitch, false,
                            ElytraFlightController.Phase.CRUISE);
                }
            }
        }
        return new Result(best, steps, System.nanoTime() - started,
                exhausted ? Status.BUDGET_EXHAUSTED : best == null ? Status.NO_SAFE_CANDIDATE : Status.COMPLETE,
                bestState, bestScore);
    }

    public enum PrefixStatus { SAFE_PREFIX, UNPROVEN }

    /** First failing check per candidate; budget counts the shared search cutoff once. */
    public record Rejections(int unknown, int collision, int damage, int budget) {}

    /** Steps contain tick-start states, as required by admission, not tick-end predictions.
     * SAFE_PREFIX is only a local preview, never a terminal safety guarantee.
     * An UNPROVEN decision is a forward, non-rocket fallback and has no admitted steps.
     */
    public record PrefixResult(ElytraFlightController.Decision decision,
            List<FlightControlAdmission.Step> prefix, int steps, long elapsedNanos,
            PrefixStatus status, PhysicsState predictedState, double score, Rejections rejections) {
        public PrefixResult { prefix = List.copyOf(prefix); }
    }

    public PrefixResult solvePrefix(List<Vector3d> lookahead, PhysicsState initial, World world,
            MovementParameters movement, float yaw, float pitch, boolean attached, Budget budget) {
        return solvePrefix(lookahead, initial, world, movement, yaw, pitch, attached, budget, null);
    }

    /** Longest known, collision/damage-free prefix wins; progress breaks length ties. */
    public PrefixResult solvePrefix(List<Vector3d> lookahead, PhysicsState initial, World world,
            MovementParameters movement, float yaw, float pitch, boolean attached, Budget budget,
            MovingTarget movingTarget) {
        Objects.requireNonNull(world);
        Objects.requireNonNull(movement);
        Objects.requireNonNull(budget);
        if (!Float.isFinite(yaw) || !Float.isFinite(pitch)) throw new IllegalArgumentException("Invalid angles");
        checked(initial.position());
        checked(initial.velocity());
        if (lookahead.isEmpty()) throw new IllegalArgumentException("Empty lookahead");
        List<Vector3d> route = lookahead.stream().map(LocalLookaheadSolver::checked)
                .sorted(java.util.Comparator.comparingDouble(p -> p.distanceSquared(initial.position()))).toList();
        long started = System.nanoTime();
        float aim = yaw;
        for (Vector3d point : route) {
            Vector3d delta = new Vector3d(point).sub(initial.position());
            if (delta.x != 0 || delta.z != 0) {
                aim = (float) Math.toDegrees(Math.atan2(-delta.x, delta.z));
                break;
            }
        }
        List<Float> yaws = new ArrayList<>();
        addYaw(yaws, aim);
        for (int i = 0; i < Math.min(3, route.size()); i++) {
            Vector3d delta = new Vector3d(route.get(i)).sub(initial.position());
            if (delta.x != 0 || delta.z != 0)
                addYaw(yaws, (float) Math.toDegrees(Math.atan2(-delta.x, delta.z)));
        }
        addYaw(yaws, yaw);
        addYaw(yaws, yaw - 30);
        addYaw(yaws, yaw + 30);
        final float forward = aim;
        yaws.removeIf(candidate -> Math.cos(Math.toRadians(candidate - forward)) <= 1.0e-7);
        float center = Math.max(-75, Math.min(60, pitch));
        float[] pitches = {center, -20, Math.max(-80, center - 15), Math.min(70, center + 15), -45, 20};
        int steps = 0, unknown = 0, collision = 0, damage = 0, cutoff = 0;
        List<FlightControlAdmission.Step> best = List.of();
        PhysicsState bestState = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        boolean exhausted = false;
        for (float candidateYaw : yaws) {
            for (float candidatePitch : pitches) {
                PhysicsState state = new PhysicsState(initial.position(), initial.velocity(), initial.onGround(),
                        initial.fallDistance(), PlayerPose.GLIDING);
                var control = new ElytraFlightController.Decision(candidateYaw, candidatePitch, false,
                        ElytraFlightController.Phase.CRUISE);
                var prefix = new ArrayList<FlightControlAdmission.Step>();
                var physics = new GlidingPhysics(candidateYaw, candidatePitch, attached, PlayerInput.NONE, movement);
                for (int tick = 0; tick < budget.horizon(); tick++) {
                    if (steps >= budget.maxSteps() || budget.maxNanos() > 0
                            && System.nanoTime() - started >= budget.maxNanos()) {
                        cutoff++;
                        exhausted = true;
                        break;
                    }
                    PhysicsResult result = physics.step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                    steps++;
                    if (!FlightSafety.knownStep(world, state, result)) { unknown++; break; }
                    if (FlightSafety.collision(result) || touchesCollision(world, result.position())
                            || state.velocity().y > .003 && result.velocity().y == 0) { collision++; break; }
                    if (FlightHazards.damage(world, state, gliding(result), DamageParameters.NONE, true).raw() > 0
                            || FlightHazards.damage(world, state, gliding(result), DamageParameters.NONE, false).raw() > 0) {
                        damage++;
                        break;
                    }
                    prefix.add(new FlightControlAdmission.Step(control, state));
                    state = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                            result.fallDistance(), PlayerPose.GLIDING);
                }
                if (!prefix.isEmpty()) {
                    double score = routeScore(route, initial.position(), state.position());
                    if (movingTarget != null) score += movingTarget.score(state, prefix.size());
                    if (prefix.size() > best.size() || prefix.size() == best.size() && score > bestScore) {
                        best = List.copyOf(prefix);
                        bestState = state;
                        bestScore = score;
                    }
                }
                if (exhausted) break;
            }
            if (exhausted) break;
        }
        var decision = best.isEmpty()
                ? new ElytraFlightController.Decision(aim, -20, false, ElytraFlightController.Phase.CRUISE)
                : best.get(0).control();
        return new PrefixResult(decision, best, steps, System.nanoTime() - started,
                best.isEmpty() ? PrefixStatus.UNPROVEN : PrefixStatus.SAFE_PREFIX, bestState, bestScore,
                new Rejections(unknown, collision, damage, cutoff));
    }

    private static boolean touchesCollision(World world, Vector3d feet) {
        double radius = PlayerPose.HALF_WIDTH + 1.0e-7;
        return world.isBoxColliding(feet.x - radius, feet.y - 1.0e-7, feet.z - radius,
                feet.x + radius, feet.y + PlayerPose.GLIDING.height() + 1.0e-7, feet.z + radius);
    }

    private static PhysicsResult gliding(PhysicsResult result) {
        return new PhysicsResult(result.position(), result.velocity(), result.collidedX(), result.collidedZ(),
                result.onGround(), result.fallDistance(), PlayerPose.GLIDING);
    }

    private static void addYaw(List<Float> yaws, float yaw) {
        float wrapped = ((yaw + 180) % 360 + 360) % 360 - 180;
        if (!yaws.contains(wrapped)) yaws.add(wrapped);
    }

    /** Arc-length progress minus cross-track distance along the ordered local polyline. */
    private static double routeScore(List<Vector3d> route, Vector3d start, Vector3d position) {
        double progress = 0, nearestDistance = Double.POSITIVE_INFINITY, score = 0;
        Vector3d from = start;
        for (Vector3d to : route) {
            Vector3d delta = new Vector3d(to).sub(from);
            double length = delta.length();
            double fraction = length == 0 ? 0 : Math.max(0, Math.min(1,
                    new Vector3d(position).sub(from).dot(delta) / (length * length)));
            double distance = position.distance(new Vector3d(delta).mul(fraction).add(from));
            if (distance < nearestDistance) {
                nearestDistance = distance;
                score = progress + fraction * length - 2 * distance;
            }
            progress += length;
            from = to;
        }
        return score;
    }

    private static Vector3d checked(Vector3d vector) {
        if (!Double.isFinite(vector.x) || !Double.isFinite(vector.y) || !Double.isFinite(vector.z))
            throw new IllegalArgumentException("Non-finite vector");
        return new Vector3d(vector);
    }
}
