package xin.bbtt.physics;

import java.util.ArrayList;
import java.util.List;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Event-time bounded full-trajectory search. UNPROVEN is not evidence of impossibility.
 * No live controller may substitute fresh feedback controls for the stored witness.
 */
public final class KnownReturnPlan {
    public static final int MAX_STEPS = 24000;
    public static final int MAX_TICKS = 600;
    private KnownReturnPlan() {}

    public record Step(PhysicsState before, PhysicsState after,
                       ElytraFlightController.Decision control, FireworkSchedule.Tick resource) {}
    public record Result(List<Step> steps, int simulatedSteps, String reason) {
        public Result { steps = List.copyOf(steps); }
        public boolean proven() { return !steps.isEmpty(); }
        /** Executed witness length, unlike simulatedSteps (all attempted policies and settling). */
        public int trajectoryTicks() { return steps.size(); }
        public int requiredRockets() {
            return proven() ? steps.get(steps.size() - 1).resource().checkpoint().consumed() : -1;
        }
    }

    /** Deterministic policy bank: climb while reversing, cruise, then unpowered descent
     * with early flares to bound airborne fall accumulation.
     * Requests cease before approach so even the minimum-lifetime schedule must be
     * quiescent at contact. Actual longer attachments require execution invalidation.
     * The budget is shared across all policies, not reset for each candidate.
     */
    public static Result preview(World world, PhysicsState initial, MovementParameters movement,
            HazardRecovery.Context resources, Vector3d anchor, int confirmationDelay) {
        if (anchor == null || !anchor.isFinite() || !initial.position().isFinite()
                || !initial.velocity().isFinite() || confirmationDelay < 0)
            return new Result(List.of(), 0, "Invalid return inputs");
        int[] flights = resources.rocketFlightDurations();
        int[] work = {0};
        Result[] result = {null};
        world.readTerrain(() -> {
            var site = VanillaGlideMode.landingSite(world, anchor.x, anchor.z, anchor.y);
            if (site == null || site.distance(anchor) > 1.0e-6) return false;
            for (float climb : new float[] {-30, -15, 0}) {
                for (double approach : new double[] {24, 40, 64}) {
                    for (int count : new int[] {0, 1, 2, 4, 8}) {
                        if (count > flights.length) continue;
                        var schedule = FireworkSchedule.confirmed(flights, count,
                                resources.cooldownTicks(), resources.guaranteedBoostTicks(), confirmationDelay);
                        var states = new ArrayList<Step>();
                        PhysicsState state = initial;
                        boolean descending = false;
                        for (int tick = 0; tick < MAX_TICKS; tick++) {
                            if (Thread.currentThread().isInterrupted() || work[0] >= MAX_STEPS) return false;
                            Vector3d delta = new Vector3d(anchor).sub(state.position());
                            double distance = Math.hypot(delta.x, delta.z);
                            float yaw = (float) Math.toDegrees(Math.atan2(-delta.x, delta.z));
                            double toward = distance < 1.0e-6 ? 0
                                    : (state.velocity().x * delta.x + state.velocity().z * delta.z) / distance;
                            if (distance < approach && toward > 0) descending = true;
                            float pitch = !descending ? (toward < .2 || state.position().y < anchor.y + 12 ? climb : 0)
                                    : (float) Math.max(10, Math.min(55,
                                            Math.toDegrees(Math.atan2(state.position().y - anchor.y, Math.max(1, distance)))));
                            if (descending && state.position().y < anchor.y + 5 && delta.y > -5)
                                pitch = -20; // dissipate descent before supported contact
                            // Flare before the incoming velocity crosses the vanilla fall-reset
                            // boundary. Store this control in the witness; execution must not
                            // replace it with feedback or restart the resource schedule.
                            if (descending && state.velocity().y < -.4) pitch = -35;
                            var resource = schedule.nextTick(!descending);
                            var decision = new ElytraFlightController.Decision(yaw, pitch, resource.useRequested(),
                                    descending ? ElytraFlightController.Phase.LANDING : ElytraFlightController.Phase.CRUISE);
                            var step = new GlidingPhysics(yaw, pitch, resource.powered(), PlayerInput.NONE, movement)
                                    .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                            work[0]++;
                            if (step.fallDistance() > movement.safeFallDistance()
                                    || !FlightSafety.knownStep(world, state, step) || step.collidedX() || step.collidedZ()
                                    || FlightHazards.damage(world, state, step, DamageParameters.NONE, true).raw() > 0
                                    || FlightHazards.damage(world, state, step, DamageParameters.NONE, false).raw() > 0) break;
                            var next = new PhysicsState(step.position(), step.velocity(), step.onGround(),
                                    step.fallDistance(), step.pose());
                            states.add(new Step(state, next, decision, resource));
                            if (step.onGround()) {
                                // Zero thrust on contact and no pending attachment: do not certify
                                // a rocket-accelerated grounded sample as a finished landing.
                                var clock = resource.checkpoint();
                                var landed = VanillaGlideMode.landingSite(world, next.position().x,
                                        next.position().z, next.position().y);
                                if (!resource.powered() && clock.boostTicks() == 0 && clock.pendingTicks() < 0
                                        && landed != null && landed.distance(next.position()) < 1.0e-6
                                        && next.position().distance(anchor) <= approach
                                        && state.velocity().y >= -.5 && state.fallDistance() <= movement.safeFallDistance()
                                        && FlightHazards.safeLanding(world, next.position())
                                        && settles(world, next, movement, work)) {
                                    result[0] = new Result(states, work[0], "Supported unpowered touchdown");
                                    return true;
                                }
                                break;
                            }
                            state = next;
                        }
                    }
                }
            }
            return false;
        });
        return result[0] == null ? new Result(List.of(), work[0], "Return policy bank unproven") : result[0];
    }

    /** Replays one already selected policy, never searches. Used only at initial admission
     * when event-time preparation advanced the physical clock. Resource history must still
     * match at the caller; every swept step and the new terminal are checked again.
     */
    static Result replay(World world, PhysicsState initial, MovementParameters movement, Result plan) {
        if (!plan.proven() || plan.steps().size() > MAX_TICKS) return new Result(List.of(), 0, "Invalid replay");
        var states = new ArrayList<Step>();
        var state = initial;
        int[] work = {0};
        for (var stored : plan.steps()) {
            var control = stored.control();
            var resource = stored.resource();
            var next = new GlidingPhysics(control.yaw(), control.pitch(), resource.powered(),
                    PlayerInput.NONE, movement).step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            work[0]++;
            if (next.fallDistance() > movement.safeFallDistance()
                    || !FlightSafety.knownStep(world, state, next) || next.collidedX() || next.collidedZ()
                    || FlightHazards.damage(world, state, next, DamageParameters.NONE, true).raw() > 0
                    || FlightHazards.damage(world, state, next, DamageParameters.NONE, false).raw() > 0) break;
            var after = FlightSafety.nextState(next);
            states.add(new Step(state, after, control, resource));
            if (next.onGround()) {
                var clock = resource.checkpoint();
                var site = VanillaGlideMode.landingSite(world, after.position().x, after.position().z, after.position().y);
                if (!resource.powered() && clock.boostTicks() == 0 && clock.pendingTicks() < 0
                        && site != null && site.distance(after.position()) < 1.0e-6
                        && state.velocity().y >= -.5 && state.fallDistance() <= movement.safeFallDistance()
                        && after.position().distance(plan.steps().get(plan.steps().size() - 1).after().position()) <= 8
                        && FlightHazards.safeLanding(world, after.position()) && settles(world, after, movement, work))
                    return new Result(states, work[0], "Replayed supported unpowered touchdown");
                break;
            }
            state = after;
        }
        return new Result(List.of(), work[0], "Fixed return replay unproven");
    }

    public static boolean launchBudget(World world, PhysicsState initial,
            MovementParameters parameters, HazardRecovery.Context resources, float yaw, float pitch, Vector3d shore) {
        PhysicsState state = initial;
        for (int tick = 0; tick < 40; tick++) {
            var next = new WalkingPhysics(PlayerInput.NONE.withJump(tick == 0), yaw, true, parameters)
                    .step(world, state, PlayerPose.HALF_WIDTH, state.pose().height());
            if (!FlightSafety.knownStep(world, state, next) || next.onGround()) return false;
            state = FlightSafety.nextState(next);
            if (state.velocity().y <= 0) break;
        }
        int[] flights = resources.rocketFlightDurations();
        var clock = FireworkSchedule.confirmed(flights, flights.length, resources.cooldownTicks(),
                resources.guaranteedBoostTicks(), 0);
        // Two physical reuse windows cover deployment and the first monitored outward
        // prefix. Reserve the resulting full reversal plus one unused safety rocket.
        for (int tick = 0; tick < 80; tick++) {
            var resource = clock.nextTick();
            var next = new GlidingPhysics(yaw, pitch, resource.powered(), PlayerInput.NONE, parameters)
                    .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (!FlightSafety.knownStep(world, state, next) || next.onGround()
                    || FlightHazards.damage(world, state, next, DamageParameters.NONE, true).raw() > 0
                    || FlightHazards.damage(world, state, next, DamageParameters.NONE, false).raw() > 0) return false;
            state = FlightSafety.nextState(next);
        }
        var checkpoint = clock.checkpoint();
        int available = flights.length - checkpoint.consumed() - 1;
        if (available < 0 || checkpoint.pendingTicks() >= 0) return false;
        int[] remaining = java.util.Arrays.copyOfRange(flights, checkpoint.consumed(),
                checkpoint.consumed() + available);
        var returnResources = new HazardRecovery.Context(remaining, checkpoint.cooldownTicks(),
                checkpoint.boostTicks() > 0, resources.memory(), checkpoint.boostTicks());
        var preview = KnownReturnPlan.preview(world, state, parameters, returnResources, shore, 0);
        if (Boolean.getBoolean("flight.return.trace")) System.err.println("launch return: " + state
                + " shore=" + shore + " remaining=" + available + " result=" + preview.reason());
        return preview.proven();
    }

    /** Both delayed glide metadata and ordinary ground braking must remain supported.
     * These are post-contact branches, not invented airborne controls or a metadata clear.
     */
    private static boolean settles(World world, PhysicsState contact, MovementParameters movement, int[] work) {
        var hold = new PlayerInput(false, false, false, false, false, true, false);
        PhysicsState delayed = contact;
        for (int delay = 0; delay <= 12; delay++) {
            PhysicsState walking = delayed;
            for (int tick = 0; tick < 40; tick++) {
                if (++work[0] > MAX_STEPS) return false;
                var next = new WalkingPhysics(hold, 0, false, movement).step(world, walking,
                        PlayerPose.HALF_WIDTH, walking.pose().height());
                if (!supported(world, walking, next)) return false;
                walking = FlightSafety.nextState(next);
                if (walking.velocity().x == 0 && walking.velocity().z == 0) break;
                if (tick == 39) return false;
            }
            if (delay == 12) return true;
            float reverse = (float) Math.toDegrees(Math.atan2(delayed.velocity().x, -delayed.velocity().z));
            var next = new GlidingPhysics(reverse, 0, false, PlayerInput.NONE, movement)
                    .step(world, delayed, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (++work[0] > MAX_STEPS || !supported(world, delayed, next)) return false;
            delayed = FlightSafety.nextState(next);
        }
        return false;
    }

    private static boolean supported(World world, PhysicsState before, PhysicsResult after) {
        if (!after.onGround() || !FlightSafety.knownStep(world, before, after)
                || !FlightHazards.safeLanding(world, after.position())
                || FlightHazards.damage(world, before, after, DamageParameters.NONE, true).raw() > 0
                || FlightHazards.damage(world, before, after, DamageParameters.NONE, false).raw() > 0) return false;
        var p = after.position();
        // Require each footprint corner to have known support, not merely one contact point.
        double radius = PlayerPose.HALF_WIDTH - 1.0e-4;
        for (double dx : new double[] {-radius, radius}) for (double dz : new double[] {-radius, radius}) {
            var site = VanillaGlideMode.landingSite(world, p.x + dx, p.z + dz, p.y);
            if (site == null || Math.abs(site.y - p.y) > 1.0e-6) return false;
        }
        return true;
    }
}
