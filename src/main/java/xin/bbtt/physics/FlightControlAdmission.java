package xin.bbtt.physics;

import xin.bbtt.MovementSync;
import java.util.List;
import java.util.ArrayList;

/** Physics-boundary admission. Preparation may search; consumption only replays fixed controls. */
public final class FlightControlAdmission {
    public record Source(long tick, long generation, PhysicsState state) {}
    public record Step(ElytraFlightController.Decision control, PhysicsState expected) {}
    public record Envelope(Object owner, Source source, List<Step> steps,
                           WaypointFlightTarget corridor, Runnable accepted, List<Step> recovery) {
        public Envelope { steps = List.copyOf(steps); recovery = List.copyOf(recovery); }
    }
    private Envelope pending;
    private Envelope active;
    private long admittedAt;
    private int cursor;
    private boolean recovering;

    public static Source snapshot(MovementSync sync) {
        synchronized (sync) {
            return new Source(sync.physicsTickVersion(), sync.getNavigationGeneration(),
                    new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), sync.fallDistance.get(), PlayerPose.GLIDING));
        }
    }

    /** Immutable controls and corridor copy; no planner references cross into the physics monitor. */
    public static Envelope prepare(MovementSync sync, Object owner, Source source,
            ElytraFlightController.Decision selected, WaypointFlightTarget route, Runnable accepted) {
        var corridor = route == null ? null : new WaypointFlightTarget(route.waypoints(), route.goal(),
                route.landed(), route.partial(), route.corridorRadius());
        var steps = new ArrayList<Step>();
        var state = source.state();
        var parameters = sync.getMovementParameters();
        boolean attached = sync.hasAttachedFirework();
        var control = selected;
        // Preparation is deliberately outside MovementSync: terminal recovery may search.
        var reserve = terminal(sync, state, selected, attached);
        for (int i = 0; i < 6; i++) {
            var result = new GlidingPhysics(control.yaw(), control.pitch(), attached, PlayerInput.NONE, parameters)
                    .step(sync.getWorld(), state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (!safe(sync, state, result, corridor)) break;
            steps.add(new Step(new ElytraFlightController.Decision(control.yaw(), control.pitch(),
                    i == 0 && selected.firework(), control.phase()), state));
            state = FlightSafety.nextState(result);
            if (result.onGround()) break;
        }
        if (!state.onGround()) {
            var continuation = terminal(sync, state, selected, attached);
            // A preview without a finite terminal continuation is not an admitted lease.
            if (continuation.isEmpty()) steps.clear();
            else steps.addAll(continuation);
        }
        return new Envelope(owner, source, steps, corridor, accepted, reserve);
    }

    private static List<Step> terminal(MovementSync sync, PhysicsState initial,
            ElytraFlightController.Decision selected, boolean attached) {
        var parameters = sync.getMovementParameters();
        var context = new HazardRecovery.Context(new int[0], 0, attached, null);
        var report = HazardRecovery.assess(sync.getWorld(), initial, parameters, context);
        var witness = report.status() == HazardRecovery.Status.RECOVERABLE
                ? HazardRecovery.controlWitness(sync.getWorld()) : null;
        var proof = HazardRecovery.revalidate(sync.getWorld(), initial, parameters, context, witness,
                new HazardRecovery.Budget(HazardRecovery.DECISION_STEP_BUDGET));
        if (proof.status() == HazardRecovery.Status.RECOVERABLE && !proof.decision().firework()) {
            var steps = terminalRollout(sync, initial, proof.decision(), attached);
            if (!steps.isEmpty()) return steps;
        }
        // NOT_EXPOSED is not a terminal proof. Search a small fixed bank even over
        // ordinary support; never turn an emergencyHold preview into an endless lease.
        for (float offset : new float[] {0, -45, 45, -90, 90, 180}) {
            for (float pitch : new float[] {25, 10, 0, -10}) {
                var control = new ElytraFlightController.Decision(selected.yaw() + offset, pitch,
                        false, ElytraFlightController.Phase.LANDING);
                var steps = terminalRollout(sync, initial, control, attached);
                if (!steps.isEmpty()) return steps;
            }
        }
        return List.of();
    }

    private static List<Step> terminalRollout(MovementSync sync, PhysicsState initial,
            ElytraFlightController.Decision control, boolean attached) {
        var steps = new ArrayList<Step>();
        var state = initial;
        for (int i = 0; i < 600; i++) {
            // Unknown attachment age guarantees only its current tick, never 600 boosted ticks.
            var result = new GlidingPhysics(control.yaw(), control.pitch(), attached && i == 0,
                    PlayerInput.NONE, sync.getMovementParameters()).step(sync.getWorld(), state,
                    PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (!safe(sync, state, result, null)) return List.of();
            steps.add(new Step(control, state));
            state = FlightSafety.nextState(result);
            if (result.onGround()) return FlightHazards.safeLanding(sync.getWorld(), result.position())
                    ? steps : List.of();
        }
        return List.of();
    }

    public void submit(Envelope envelope) { pending = envelope; }
    public void revoke(Object owner) {
        if (pending != null && pending.owner() == owner) pending = null;
        if (active != null && active.owner() == owner) active = null;
    }
    public void clear() { pending = null; active = null; }

    /** Called under MovementSync only. Never takes navigation/controller locks or invokes search. */
    public void consume(MovementSync sync) {
        if (!sync.isGliding()) {
            clear();
            return;
        }
        if (pending != null) {
            var candidate = pending;
            pending = null;
            long age = sync.physicsTickVersion() - candidate.source().tick();
            if (candidate.source().generation() == sync.getNavigationGeneration() && age >= 0 && age <= 8
                    && !candidate.steps().isEmpty() && revalidate(sync, candidate, 0)) {
                active = candidate;
                cursor = 0;
                recovering = false;
                admittedAt = sync.physicsTickVersion();
                // Actions are queued only at acceptance, immediately before the packet/input batch.
                // Recheck the powered branch as well: rejecting thrust must reject its action,
                // not silently rely on the unpowered continuation proof.
                if (!candidate.steps().get(0).control().firework() || poweredSafe(sync, candidate))
                    candidate.accepted().run();
            } else if (active == null && candidate.source().generation() == sync.getNavigationGeneration()
                    && age >= 0 && age <= 8 && !candidate.recovery().isEmpty()) {
                var reserve = new Envelope(candidate.owner(), candidate.source(), candidate.recovery(), null,
                        () -> {}, List.of());
                if (revalidate(sync, reserve, 0)) {
                    active = reserve;
                    cursor = 0;
                    admittedAt = sync.physicsTickVersion();
                    recovering = true;
                } else abandon(sync);
            } else if (active == null && candidate.source().generation() == sync.getNavigationGeneration()
                    && age >= 0 && age <= 8) {
                abandon(sync);
            }
        }
        if (active == null) return;
        if (active.source().generation() != sync.getNavigationGeneration() || !sync.isGliding()) {
            clear();
            return;
        }
        cursor = (int) (sync.physicsTickVersion() - admittedAt);
        if (cursor >= active.steps().size() || !revalidate(sync, active, cursor)) {
            // Never silently keep an expired navigation command. Normal airborne recovery
            // owns the next planning tick; do not fabricate touchdown or disable gliding.
            if (!recovering && !active.recovery().isEmpty()) {
                var reserve = new Envelope(active.owner(), active.source(), active.recovery(), null,
                        () -> {}, List.of());
                if (revalidate(sync, reserve, 0)) {
                    active = reserve;
                    admittedAt = sync.physicsTickVersion();
                    cursor = 0;
                    recovering = true;
                } else {
                    abandon(sync);
                    return;
                }
            } else {
                abandon(sync);
                return;
            }
        }
        var control = active.steps().get(cursor).control();
        sync.setYawToward(control.yaw());
        sync.pitch.set(control.pitch());
    }

    private void abandon(MovementSync sync) {
        active = null;
        sync.clearInput();
        // No safe witness remains (terrain/state changed). Explicit last-resort descent,
        // not a claim of safety and not indefinite reuse of the expired route heading.
        sync.setYawToward(LowAltitudePlanner.yaw(sync.velocity.get()));
        sync.pitch.set(90f);
    }

    private static boolean poweredSafe(MovementSync sync, Envelope envelope) {
        var state = new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), sync.fallDistance.get(), PlayerPose.GLIDING);
        for (int i = 0; i < Math.min(6, envelope.steps().size()); i++) {
            var control = envelope.steps().get(i).control();
            var result = new GlidingPhysics(control.yaw(), control.pitch(), true, PlayerInput.NONE,
                    sync.getMovementParameters()).step(sync.getWorld(), state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (!safe(sync, state, result, i < 6 ? envelope.corridor() : null)) return false;
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance(), result.pose());
        }
        return true;
    }

    private static boolean revalidate(MovementSync sync, Envelope envelope, int index) {
        var state = new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), sync.fallDistance.get(), PlayerPose.GLIDING);
        // Fixed maximum of six physics steps, selected controls only, no candidate search.
        for (int i = index; i < Math.min(index + 6, envelope.steps().size()); i++) {
            var control = envelope.steps().get(i).control();
            var result = new GlidingPhysics(control.yaw(), control.pitch(), sync.hasAttachedFirework(),
                    PlayerInput.NONE, sync.getMovementParameters()).step(sync.getWorld(), state,
                    PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (!safe(sync, state, result, i < 6 ? envelope.corridor() : null)) return false;
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance(), result.pose());
            if (result.onGround()) break;
        }
        return true;
    }

    private static boolean safe(MovementSync sync, PhysicsState before, PhysicsResult after, WaypointFlightTarget corridor) {
        var p = after.position();
        return p.isFinite() && sync.getWorld().chunkLoaded(((int)Math.floor(p.x)) >> 4, ((int)Math.floor(p.z)) >> 4)
                && !after.collidedX() && !after.collidedZ()
                && FlightHazards.damage(sync.getWorld(), before, after, DamageParameters.NONE, true).raw() == 0
                && FlightHazards.damage(sync.getWorld(), before, after, DamageParameters.NONE, false).raw() == 0
                && (corridor == null || corridor.contains(before.position(), p));
    }
}
