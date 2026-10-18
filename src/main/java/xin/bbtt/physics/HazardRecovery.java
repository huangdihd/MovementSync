package xin.bbtt.physics;

import org.joml.Vector3d;
import xin.bbtt.world.World;
import xin.bbtt.movements.LandingMemory;
import xin.bbtt.physics.ElytraFlightController.Decision;
import xin.bbtt.physics.ElytraFlightController.Phase;

/** Finite, collision-checked landing reserve using the actual remaining rocket inventory.
 * A range estimate alone is never proof; both actual and stalled states must touch safe support.
 */
public final class HazardRecovery {
    private static final int MAX_TICKS = 600;
    /** Hard physics-step work cap; exhaustion is UNPROVEN, never permission to fly. */
    public static final class Budget {
        private final int limit;
        private int steps;
        public Budget(int limit) {
            if (limit < 0) throw new IllegalArgumentException("Negative proof budget");
            this.limit = limit;
        }
        public int steps() { return steps; }
        public boolean exhausted() { return steps >= limit; }
        private boolean take() { if (exhausted()) return false; steps++; return true; }
    }
    public static final int DECISION_STEP_BUDGET = 24_000;
    // Controls are search hints only. Every use repeats the entire terminal proof
    // in current terrain with exact current state, inventory and cooldown.
    private record Hint(float yaw, float pitch, boolean powered) {}
    private static final class Hints {
        private java.lang.ref.WeakReference<World> world = new java.lang.ref.WeakReference<>(null);
        private final Hint[] controls = new Hint[2];
    }
    private static final ThreadLocal<Hints> HINTS = ThreadLocal.withInitial(Hints::new);

    private static Hint[] hints(World world) {
        Hints hints = HINTS.get();
        if (hints.world.get() != world) {
            hints.world = new java.lang.ref.WeakReference<>(world);
            java.util.Arrays.fill(hints.controls, null);
        }
        return hints.controls;
    }
    private static final float[] PITCHES = {0, -10, -30, 10, 25};

    /** Controls only, never cached safety. Every reuse must replay both terminal proofs. */
    public static final class ControlWitness {
        private final Hint actual, stalled;
        private ControlWitness(Hint actual, Hint stalled) {
            this.actual = actual;
            this.stalled = stalled;
        }
    }

    public static ControlWitness controlWitness() {
        Hint[] hints = HINTS.get().controls;
        return hints[0] == null || hints[1] == null ? null : new ControlWitness(hints[0], hints[1]);
    }

    /** Search preferences belong to one world; unrelated sessions must not bias ranking. */
    public static ControlWitness controlWitness(World world) {
        hints(world);
        return controlWitness();
    }

    /** Independent bounded replay, not a second branching search. Current resources,
     * motion and swept terrain must still prove actual AND stalled safe touchdown. */
    public static Report revalidate(World world, PhysicsState state, MovementParameters movement,
            Context context, ControlWitness witness, Budget budget) {
        if (witness == null) return new Report(Status.UNPROVEN);
        Report[] report = {new Report(Status.UNPROVEN)};
        world.readTerrain(() -> {
            if (!state.position().isFinite() || !state.velocity().isFinite()
                    || !Double.isFinite(state.fallDistance())) return false;
            PhysicsState glide = new PhysicsState(state.position(), state.velocity(), state.onGround(),
                    state.fallDistance(), PlayerPose.GLIDING);
            PhysicsState stalled = new PhysicsState(state.position(),
                    new Vector3d(0, Math.min(0, state.velocity().y), 0), false,
                    state.fallDistance(), PlayerPose.GLIDING);
            Hint a = witness.actual, s = witness.stalled;
            Exit actual = rollout(world, glide, movement, a.yaw(), a.pitch(), a.powered(), context, budget);
            if (actual == null) return false;
            Exit reserve = rollout(world, stalled, movement, s.yaw(), s.pitch(), s.powered(), context, budget);
            if (reserve == null) return false;
            report[0] = new Report(Status.RECOVERABLE, "Revalidated actual and stalled control witness",
                    actual.decision(), actual.target());
            return true;
        });
        return report[0];
    }

    /** Immutable inventory snapshot. Unknown attached age credits only one tick. */
    public record Context(int[] rocketFlightDurations, int cooldownTicks, boolean attached, LandingMemory memory,
                          long guaranteedBoostTicks) {
        public Context(int[] flights, int cooldown, boolean attached, LandingMemory memory) {
            this(flights, cooldown, attached, memory, attached ? 1 : 0);
        }
        public static final Context EMPTY = new Context(new int[0], 0, false, null);
        public Context {
            rocketFlightDurations = rocketFlightDurations.clone();
            if (cooldownTicks < 0) throw new IllegalArgumentException("Negative rocket cooldown");
        }
        @Override public int[] rocketFlightDurations() { return rocketFlightDurations.clone(); }
        /** Reserve consumed inventory while retaining a newly fired rocket's minimum lifetime. */
        public Context afterTicks(int ticks, boolean firework) {
            if (firework && (attached || cooldownTicks > 0 || rocketFlightDurations.length == 0))
                throw new IllegalArgumentException("Rocket is unavailable");
            int[] remaining = firework ? java.util.Arrays.copyOfRange(rocketFlightDurations, 1,
                    rocketFlightDurations.length) : rocketFlightDurations;
            long boost = Math.max(0, (firework ? GlideRange.minimumBoostTicks(rocketFlightDurations[0])
                    : guaranteedBoostTicks) - ticks);
            return new Context(remaining, Math.max(0, (firework ? 40 : cooldownTicks) - ticks),
                    boost > 0, memory, boost);
        }
    }
    public enum Status { NOT_EXPOSED, RECOVERABLE, UNPROVEN }
    public record Report(Status status, String reason, Decision decision, Vector3d target) {
        public Report { target = target == null ? null : new Vector3d(target); }
        @Override public Vector3d target() { return target == null ? null : new Vector3d(target); }
        public Report(Status status) { this(status, status == Status.UNPROVEN
                ? "No finite safe supported landing proven" : "No hazard below loaded support", null, null); }
        public boolean admissible() { return status != Status.UNPROVEN; }
    }
    private record Exit(Decision decision, Vector3d target) {}

    private HazardRecovery() {}

    /** NOT_EXPOSED means only that the loaded column has no hazard before solid
     * support/world bottom; it does not certify indefinite flight or void safety.
     * Call on each executable state, not just the end of a look-ahead horizon.
     */
    public static Report assess(World world, PhysicsState state, MovementParameters movement) {
        return assess(world, state, movement, Context.EMPTY);
    }

    public static Report assess(World world, PhysicsState state, MovementParameters movement, Context context) {
        return assess(world, state, movement, context, new Budget(DECISION_STEP_BUDGET));
    }

    public static Report assess(World world, PhysicsState state, MovementParameters movement, Context context,
            Budget budget) {
        Report[] report = new Report[1];
        world.readTerrain(() -> {
            report[0] = assessLocked(world, state, movement, context, budget);
            return true;
        });
        return report[0];
    }

    /** Improve a proven rescue toward the requested destination without spending its
     * proof budget or discarding it. The old stalled-state reserve remains valid;
     * each replacement actual trajectory must independently reach loaded support.
     * No destination preference is applied while a rocket is unavailable now. */
    public static Report assessToward(World world, PhysicsState state, MovementParameters movement,
            Context context, Vector3d destination, java.util.function.Predicate<Decision> allowed) {
        Report fallback = assess(world, state, movement, context);
        if (fallback.status() != Status.RECOVERABLE || destination == null || !destination.isFinite()
                || context.attached() || context.cooldownTicks() > 0
                || context.rocketFlightDurations().length == 0) return fallback;
        Report[] best = {fallback};
        Budget budget = new Budget(DECISION_STEP_BUDGET);
        world.readTerrain(() -> {
            float heading = LowAltitudePlanner.yaw(new Vector3d(destination).sub(state.position()));
            for (float offset : new float[] {0, -45, 45, -90, 90}) {
                for (boolean powered : new boolean[] {false, true}) {
                    for (float pitch : PITCHES) {
                        if (budget.exhausted()) return true;
                        Exit exit = rollout(world, state, movement, heading + offset, pitch, powered, context, budget);
                        if (exit != null && exit.target().distance(destination) < best[0].target().distance(destination)
                                && allowed.test(exit.decision())) {
                            best[0] = new Report(Status.RECOVERABLE,
                                    "Target-nearer loaded supported landing; original stalled reserve retained",
                                    exit.decision(), exit.target());
                        }
                    }
                }
            }
            return true;
        });
        return best[0];
    }

    private static Report assessLocked(World world, PhysicsState state, MovementParameters movement, Context context, Budget budget) {
        hints(world);
        if (!state.position().isFinite() || !state.velocity().isFinite()
                || !Double.isFinite(state.fallDistance())) return new Report(Status.UNPROVEN);
        Vector3d feet = state.position();
        if (feet.y < World.getMinWorldY() || feet.y + PlayerPose.GLIDING.height() > World.getMaxWorldY())
            return new Report(Status.UNPROVEN);
        double radius = PlayerPose.HALF_WIDTH;
        boolean exposed = false;
        // Check each body-overlapped column separately: a ledge under one corner
        // must not hide lava under the other corner.
        for (int x = (int) Math.floor(feet.x - radius); x <= Math.floor(feet.x + radius); x++) {
            for (int z = (int) Math.floor(feet.z - radius); z <= Math.floor(feet.z + radius); z++) {
                if (!world.chunkLoaded(x >> 4, z >> 4)) return new Report(Status.UNPROVEN);
                for (int y = (int) Math.floor(feet.y); y >= World.getMinWorldY(); y--) {
                    Vector3d cell = new Vector3d(x + .5, y + .01, z + .5);
                    PhysicsState probe = new PhysicsState(cell, new Vector3d(), false, 0, PlayerPose.GLIDING);
                    PhysicsResult still = new PhysicsResult(cell, new Vector3d(), false, false, false, 0,
                            PlayerPose.GLIDING);
                    if (hazard(world, probe, still)) { exposed = true; break; }
                    if (world.isBoxColliding(x, y, z, x + 1, y + 1, z + 1)) break;
                }
            }
        }
        if (!exposed) return new Report(Status.NOT_EXPOSED);
        PhysicsState glide = new PhysicsState(feet, state.velocity(), state.onGround(),
                state.fallDistance(), PlayerPose.GLIDING);
        // A wall can erase horizontal motion without erasing an existing descent.
        PhysicsState stalled = new PhysicsState(feet, new Vector3d(0, Math.min(0, state.velocity().y), 0),
                false, state.fallDistance(), PlayerPose.GLIDING);
        float heading = LowAltitudePlanner.yaw(state.velocity());
        Exit actual = recover(world, glide, movement, heading, context, budget, 0);
        if (actual == null) return unproven(context, "actual flight");
        Exit reserve = recover(world, stalled, movement, heading, context, budget, 1);
        return reserve == null ? unproven(context, "stalled flight") : new Report(Status.RECOVERABLE,
                "Actual and stalled trajectories reach known safe support with remaining rockets",
                actual.decision(), actual.target());
    }

    private static Report unproven(Context context, String branch) {
        String resources = context.rocketFlightDurations().length == 0 && context.guaranteedBoostTicks() == 0
                ? "no fireworks remaining" : "rockets=" + context.rocketFlightDurations().length
                + ", cooldown=" + context.cooldownTicks() + ", guaranteedBoost=" + context.guaranteedBoostTicks();
        return new Report(Status.UNPROVEN, "No reachable safe landing proven for " + branch + " (" + resources
                + "); bounded corridor rollout exhausted; obstruction versus insufficient range unresolved", null, null);
    }

    private static Exit recover(World world, PhysicsState initial, MovementParameters movement,
            float heading, Context context, Budget budget, int branch) {
        Hint hint = hints(world)[branch];
        if (hint != null) {
            Exit exit = rollout(world, initial, movement, hint.yaw(), hint.pitch(), hint.powered(), context, budget);
            if (exit != null) return exit;
        }
        // Try eight local bearings before scanning every loaded landing column.
        // Their successes are still complete collision-checked terminal rollouts.
        for (int bearing = 0; bearing < 8 && !budget.exhausted(); bearing++) {
            Exit exit = direction(world, initial, movement, heading + bearing * 45, context, budget, branch);
            if (exit != null) return exit;
        }
        if (budget.exhausted()) return null;
        // Reuse the End landing candidate search, but accept only real world terminal rollouts.
        Exit[] found = new Exit[1];
        int[] attempts = {0};
        java.util.function.Predicate<Vector3d> reachable = site -> {
            if (budget.exhausted() || ++attempts[0] > 32) return false;
            float yaw = LowAltitudePlanner.yaw(new Vector3d(site).sub(initial.position()));
            found[0] = direction(world, initial, movement, yaw, context, budget, branch);
            return found[0] != null;
        };
        if (context.memory() != null) {
            Vector3d remembered = context.memory().nearest(initial.position());
            // Memory is a candidate, never permission to cross unknown terrain.
            if (remembered != null && reachable.test(remembered)) return found[0];
        }
        new LandingSearch().selectReachable(world, initial, context.rocketFlightDurations(),
                context.attached(), movement, LandingSearch.BUDGET, reachable);
        if (found[0] != null) return found[0];
        return null;
    }

    private static Exit direction(World world, PhysicsState initial, MovementParameters movement,
            float yaw, Context context, Budget budget, int branch) {
        int[] rockets = context.rocketFlightDurations();
        for (boolean powered : new boolean[] {false, true}) {
            if (powered && rockets.length == 0 && !context.attached()) continue;
            for (float pitch : PITCHES) {
                if (budget.exhausted()) return null;
                Exit exit = rollout(world, initial, movement, yaw, pitch, powered, context, budget);
                if (exit != null) {
                    hints(world)[branch] = new Hint(yaw, pitch, powered);
                    return exit;
                }
            }
        }
        return null;
    }

    private static Exit rollout(World world, PhysicsState initial, MovementParameters movement,
            float yaw, float pitch, boolean powered, Context context, Budget budget) {
        FlightSafety safety = new FlightSafety(movement, DamageParameters.NONE, 20);
        int[] rockets = context.rocketFlightDurations();
        PhysicsState state = initial;
        int used = 0, cooldown = context.cooldownTicks();
        long boost = context.guaranteedBoostTicks();
        Decision first = null;
        for (int tick = 0; tick < MAX_TICKS; tick++) {
            if (Thread.currentThread().isInterrupted() || !budget.take()) return null;
            boolean fire = powered && boost == 0 && cooldown == 0 && used < rockets.length;
            if (fire) { boost = GlideRange.minimumBoostTicks(rockets[used++]); cooldown = 40; }
            boolean attached = boost > 0;
            if (first == null) first = new Decision(yaw, pitch, fire, Phase.LANDING);
            PhysicsResult result = new GlidingPhysics(yaw, pitch, attached, PlayerInput.NONE, movement)
                    .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (!FlightSafety.knownStep(world, state, result) || hazard(world, state, result)
                    || result.collidedX() || result.collidedZ()
                    || safety.collisionDamage(world, state, result, yaw, pitch, attached).raw() > 0) break;
            if (result.onGround()) {
                Vector3d p = result.position();
                if (FlightHazards.safeLanding(world, p)
                        && !world.isBoxColliding(p.x - PlayerPose.HALF_WIDTH, p.y, p.z - PlayerPose.HALF_WIDTH,
                                p.x + PlayerPose.HALF_WIDTH, p.y + PlayerPose.STANDING.height(), p.z + PlayerPose.HALF_WIDTH))
                    return new Exit(first, p);
                break;
            }
            state = FlightSafety.nextState(result);
            boost = Math.max(0, boost - 1);
            cooldown = Math.max(0, cooldown - 1);
        }
        return null;
    }

    private static boolean hazard(World world, PhysicsState state, PhysicsResult result) {
        return FlightHazards.damage(world, state, result, DamageParameters.NONE, true).raw() > 0
                || FlightHazards.damage(world, state, result, DamageParameters.NONE, false).raw() > 0;
    }
}
