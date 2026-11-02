package xin.bbtt.physics;

import xin.bbtt.MovementSync;
import java.util.List;
import java.util.ArrayList;

/** Physics-boundary admission. Preparation may search; consumption only replays fixed controls. */
public final class FlightControlAdmission {
    public enum Mode { LOCAL, LOCAL_SOLVER, DIRECT, EXTENSION, TRANSITION }
    public record Source(long tick, long generation, PhysicsState state, long epoch) {
        public Source(long tick, long generation, PhysicsState state) { this(tick, generation, state, 0); }
    }
    public record Failure(long tick, long generation, long epoch, String reason, String responsibleOwner) {}
    private Failure failure;
    private Object owner;
    private Mode mode;
    private long epoch;
    private final java.util.Set<Envelope> consumed = java.util.Collections.newSetFromMap(new java.util.WeakHashMap<>());

    /** Same-owner/mode renewals retain the token; handoffs invalidate in-flight work. */
    public void claim(Object nextOwner, Mode nextMode) {
        if (owner == nextOwner && mode == nextMode) return;
        clear();
        owner = nextOwner;
        mode = nextMode;
    }
    public long epoch() { return epoch; }
    public Failure failure() { return failure; }
    public record Step(ElytraFlightController.Decision control, PhysicsState expected) {}
    /** Planner acknowledgement is immediate or next-tick. An attachment still absent at
     * this request-relative tick is a rejection; any later attachment is re-proved by the
     * ordinary attached-rocket replay, never trusted from the rejected hand-over. */
    public static final int REJECTED_ROCKET_OBSERVATION_TICKS = 3;
    public record ReturnWitness(KnownReturnPlan.Result plan, int[] inventory,
                                java.util.function.Consumer<ElytraFlightController.Decision> action) {
        public ReturnWitness { inventory = inventory.clone(); }
        @Override public int[] inventory() { return inventory.clone(); }
    }
    /** Request-scoped state: lifetime remains request-relative across prefix and reserve. */
    private static final class RejectedRocketTermination {
        private final FireworkSchedule.LifetimeBounds lifetime;
        private long requestedAt = Long.MIN_VALUE;
        private boolean attachmentObserved;
        private boolean rejectionConfirmed;
        private int[] publicationInventory;
        // Unpowered reserve proved from the observed-rejection state; empty when the
        // rocket controls themselves must stay safe without thrust.
        private List<Step> handOver = List.of();

        private RejectedRocketTermination(FireworkSchedule.LifetimeBounds lifetime) {
            this.lifetime = lifetime;
        }
    }
    public record Envelope(Object owner, Source source, List<Step> steps,
                           WaypointFlightTarget corridor, Runnable accepted, List<Step> recovery,
                           List<Step> handoff, ReturnWitness witness, boolean requireAirborne,
                           FireworkSchedule.LifetimeBounds rocketLifetime,
                           RejectedRocketTermination rejectedRocketTermination,
                           boolean preserveFallMargin) {
        public Envelope(Object owner, Source source, List<Step> steps,
                WaypointFlightTarget corridor, Runnable accepted, List<Step> recovery,
                List<Step> handoff, ReturnWitness witness, boolean requireAirborne,
                FireworkSchedule.LifetimeBounds rocketLifetime,
                RejectedRocketTermination rejectedRocketTermination) {
            this(owner, source, steps, corridor, accepted, recovery, handoff, witness,
                    requireAirborne, rocketLifetime, rejectedRocketTermination, false);
        }
        public Envelope(Object owner, Source source, List<Step> steps,
                WaypointFlightTarget corridor, Runnable accepted, List<Step> recovery,
                List<Step> handoff, ReturnWitness witness, boolean requireAirborne,
                FireworkSchedule.LifetimeBounds rocketLifetime) {
            this(owner, source, steps, corridor, accepted, recovery, handoff, witness,
                    requireAirborne, rocketLifetime, false);
        }
        public Envelope(Object owner, Source source, List<Step> steps,
                WaypointFlightTarget corridor, Runnable accepted, List<Step> recovery,
                List<Step> handoff, ReturnWitness witness, boolean requireAirborne,
                FireworkSchedule.LifetimeBounds rocketLifetime, boolean preserveFallMargin) {
            this(owner, source, steps, corridor, accepted, recovery, handoff, witness,
                    requireAirborne, rocketLifetime,
                    rocketLifetime == null ? null : new RejectedRocketTermination(rocketLifetime),
                    preserveFallMargin);
        }
        public Envelope(Object owner, Source source, List<Step> steps, WaypointFlightTarget corridor,
                Runnable accepted, List<Step> recovery, ReturnWitness witness,
                boolean requireAirborne) {
            this(owner, source, steps, corridor, accepted, recovery, List.of(), witness,
                    requireAirborne, null, null);
        }
        public Envelope(Object owner, Source source, List<Step> steps, WaypointFlightTarget corridor,
                Runnable accepted, List<Step> recovery, List<Step> handoff, ReturnWitness witness,
                boolean requireAirborne) {
            this(owner, source, steps, corridor, accepted, recovery, handoff, witness,
                    requireAirborne, null, null);
        }
        public Envelope(Object owner, Source source, List<Step> steps, WaypointFlightTarget corridor,
                Runnable accepted, List<Step> recovery, ReturnWitness witness) {
            this(owner, source, steps, corridor, accepted, recovery, List.of(), witness,
                    false, null, null);
        }
        public Envelope(Object owner, Source source, List<Step> steps, WaypointFlightTarget corridor,
                Runnable accepted, List<Step> recovery) {
            this(owner, source, steps, corridor, accepted, recovery, List.of(), null,
                    false, null, null);
        }
        public Envelope {
            steps = List.copyOf(steps);
            recovery = List.copyOf(recovery);
            handoff = List.copyOf(handoff);
            if (handoff.stream().anyMatch(step -> step.control().firework()))
                throw new IllegalArgumentException("Proof-only handoff cannot request rockets");
            boolean requestsRocket = !steps.isEmpty() && steps.get(0).control().firework();
            if (rocketLifetime != null && !requestsRocket)
                throw new IllegalArgumentException("Lifetime bounds require a rocket request");
        }
    }

    /** Entire event-time episode; resource actions are dispatched at their own checked tick. */
    public static Envelope knownReturn(Object owner, Source source, KnownReturnPlan.Result plan,
            int[] inventory, java.util.function.Consumer<ElytraFlightController.Decision> action) {
        return new Envelope(owner, source, plan.steps().stream()
                .map(step -> new Step(step.control(), step.before())).toList(), null, () -> {}, List.of(),
                List.of(), new ReturnWitness(plan, inventory, action), false, null, null, true);
    }
    private Envelope pending;
    private Envelope active;
    private long admittedAt;
    private TickControlBatch batch;
    private int cursor;
    private boolean recovering;
    private long lastReturnActionTick = Long.MIN_VALUE;
    private Source emergencySource;
    private float emergencyYaw;
    private WaypointFlightTarget emergencyCorridor;
    private boolean emergencyCheckpointRevoked;
    /** Source identity and corridor form the in-process capability issued to one worker. */
    private record RevokedEmergencyCheckpoint(Source source, WaypointFlightTarget corridor) {}
    private final List<RevokedEmergencyCheckpoint> revokedEmergencyCheckpoints = new ArrayList<>();

    /** Copy the solver's exact executable prefix and fixed reserve; never search for a terminal. */
    public static Envelope createPrefix(Object owner, Source source, List<Step> prefix,
            List<Step> recovery, WaypointFlightTarget route, Runnable accepted) {
        return createPrefix(owner, source, prefix, recovery, List.of(), route, accepted, false);
    }

    /** Moving pursuit prefixes must remain airborne when replayed from a newer live state. */
    public static Envelope createPrefix(Object owner, Source source, List<Step> prefix,
            List<Step> recovery, WaypointFlightTarget route, Runnable accepted,
            boolean requireAirborne) {
        return createPrefix(owner, source, prefix, recovery, List.of(), route, accepted,
                requireAirborne);
    }

    /** Handoff extends only the live proof window; it is never part of an executable batch. */
    public static Envelope createPrefix(Object owner, Source source, List<Step> prefix,
            List<Step> recovery, List<Step> handoff, WaypointFlightTarget route, Runnable accepted,
            boolean requireAirborne) {
        return createPrefix(owner, source, prefix, recovery, handoff, route, accepted,
                requireAirborne, null);
    }

    public static Envelope createPrefix(Object owner, Source source, List<Step> prefix,
            List<Step> recovery, List<Step> handoff, WaypointFlightTarget route, Runnable accepted,
            boolean requireAirborne, FireworkSchedule.LifetimeBounds rocketLifetime) {
        return createPrefix(owner, source, prefix, recovery, handoff, route, accepted,
                requireAirborne, rocketLifetime, false);
    }

    /** Known-return episodes keep a strict airborne fall margin, independent of contact damage. */
    public static Envelope createPrefix(Object owner, Source source, List<Step> prefix,
            List<Step> recovery, List<Step> handoff, WaypointFlightTarget route, Runnable accepted,
            boolean requireAirborne, FireworkSchedule.LifetimeBounds rocketLifetime,
            boolean preserveFallMargin) {
        var corridor = route == null ? null : route.corridorSnapshot();
        if (recovery.stream().anyMatch(step -> step.control().firework()))
            throw new IllegalArgumentException("Fixed recovery cannot request rockets");
        if (handoff.stream().anyMatch(step -> step.control().firework()))
            throw new IllegalArgumentException("Proof-only handoff cannot request rockets");
        if (route == null && !prefix.isEmpty()
                && recovery.stream().anyMatch(step -> step.control().pitch() > 0
                        || Math.cos(Math.toRadians(step.control().yaw() - prefix.get(0).control().yaw())) <= 0))
            recovery = List.of(); // A targetless reserve is not permission for reverse descent.
        if (recovery.isEmpty()) handoff = List.of();
        return new Envelope(owner, source, prefix, corridor, accepted, recovery, handoff, null,
                requireAirborne, rocketLifetime, preserveFallMargin);
    }

    /** Caller holds sync. Invalid/stale publication never displaces the active prefix. */
    public boolean publishPrefix(MovementSync sync, Object publisher, Source source, List<Step> prefix,
            List<Step> recovery, WaypointFlightTarget route, Runnable accepted) {
        return publishPrefix(sync, publisher, source, prefix, recovery, List.of(), route, accepted,
                false);
    }

    /** Caller holds sync. Invalid/stale publication never displaces the active prefix. */
    public boolean publishPrefix(MovementSync sync, Object publisher, Source source, List<Step> prefix,
            List<Step> recovery, WaypointFlightTarget route, Runnable accepted,
            boolean requireAirborne) {
        return publishPrefix(sync, publisher, source, prefix, recovery, List.of(), route, accepted,
                requireAirborne);
    }

    /** Caller holds sync. Invalid/stale publication never displaces the active prefix. */
    public boolean publishPrefix(MovementSync sync, Object publisher, Source source, List<Step> prefix,
            List<Step> recovery, List<Step> handoff, WaypointFlightTarget route, Runnable accepted,
            boolean requireAirborne) {
        return publishPrefix(sync, publisher, source, prefix, recovery, handoff, route, accepted,
                requireAirborne, null);
    }

    public boolean publishPrefix(MovementSync sync, Object publisher, Source source, List<Step> prefix,
            List<Step> recovery, List<Step> handoff, WaypointFlightTarget route, Runnable accepted,
            boolean requireAirborne, FireworkSchedule.LifetimeBounds rocketLifetime) {
        return publishPrefix(sync, publisher, source, prefix, recovery, handoff, route, accepted,
                requireAirborne, rocketLifetime, false);
    }

    public boolean publishPrefix(MovementSync sync, Object publisher, Source source, List<Step> prefix,
            List<Step> recovery, List<Step> handoff, WaypointFlightTarget route, Runnable accepted,
            boolean requireAirborne, FireworkSchedule.LifetimeBounds rocketLifetime,
            boolean preserveFallMargin) {
        return publishPrefix(sync, publisher, source, prefix, recovery, handoff, route, route, accepted,
                requireAirborne, rocketLifetime, preserveFallMargin);
    }

    /** Publish an execution witness under the navigation route that owns the request.
     * The witness may be an exact replay corridor and therefore need not compare equal
     * to the live navigation corridor used for cancellation and ownership. */
    public boolean publishPrefix(MovementSync sync, Object publisher, Source source, List<Step> prefix,
            List<Step> recovery, List<Step> handoff, WaypointFlightTarget route,
            WaypointFlightTarget authorityRoute, Runnable accepted, boolean requireAirborne,
            FireworkSchedule.LifetimeBounds rocketLifetime, boolean preserveFallMargin) {
        return publishPrefix(sync, publisher, source, prefix, recovery, handoff, route, authorityRoute,
                accepted, requireAirborne, rocketLifetime, preserveFallMargin, List.of());
    }

    /** A rocket request may carry the unpowered reserve that takes over once rejection
     * is observed; it is replayed from the live state like any fixed reserve. */
    public boolean publishPrefix(MovementSync sync, Object publisher, Source source, List<Step> prefix,
            List<Step> recovery, List<Step> handoff, WaypointFlightTarget route,
            WaypointFlightTarget authorityRoute, Runnable accepted, boolean requireAirborne,
            FireworkSchedule.LifetimeBounds rocketLifetime, boolean preserveFallMargin,
            List<Step> rejectionHandOver) {
        if (mode != Mode.LOCAL_SOLVER || owner != publisher || !currentSource(sync, source)
                || !matchesEmergencyCheckpoint(source, authorityRoute)) return false;
        if (rejectionHandOver.stream().anyMatch(step -> step.control().firework()))
            throw new IllegalArgumentException("Rejected-rocket hand-over cannot request rockets");
        var envelope = createPrefix(publisher, source, prefix, recovery, handoff, route, accepted,
                requireAirborne, rocketLifetime, preserveFallMargin);
        if (envelope.rejectedRocketTermination() != null) {
            envelope.rejectedRocketTermination().publicationInventory =
                    sync.inventoryManager.getHotbarFireworkFlightDurations();
            envelope.rejectedRocketTermination().handOver = List.copyOf(rejectionHandOver);
        }
        submit(envelope);
        return true;
    }

    public boolean registerForwardEmergency(MovementSync sync, Object publisher, Source source,
            float yaw) {
        return registerForwardEmergency(sync, publisher, source, yaw, null);
    }

    /** Caller holds sync. Heading and corridor are one immutable physical checkpoint. */
    public boolean registerForwardEmergency(MovementSync sync, Object publisher, Source source,
            float yaw, WaypointFlightTarget route) {
        pruneRevokedEmergencyCheckpoints(sync);
        if (mode != Mode.LOCAL_SOLVER || owner != publisher || !currentSource(sync, source)
                || !Float.isFinite(yaw) || revokedEmergencyCheckpoint(source, route)
                || emergencySource != null && source.tick() < emergencySource.tick()) return false;
        if (emergencySource != null && !sameEmergencyCorridor(emergencyCorridor, route))
            revokeEmergencyCheckpoint(emergencySource, emergencyCorridor);
        emergencySource = source;
        emergencyYaw = yaw;
        emergencyCorridor = route == null ? null : route.corridorSnapshot();
        emergencyCheckpointRevoked = false;
        return true;
    }

    /** A worker may refine its exact checkpoint. A newer observation of the same
     * corridor may authorize publication, but the older worker cannot overwrite its heading.
     */
    public boolean refineForwardEmergency(MovementSync sync, Object publisher, Source source,
            float yaw, WaypointFlightTarget route) {
        if (mode != Mode.LOCAL_SOLVER || owner != publisher || !currentSource(sync, source)
                || !Float.isFinite(yaw) || !matchesEmergencyCheckpoint(source, route)) return false;
        if (emergencySource == source) emergencyYaw = yaw;
        return true;
    }

    /** Move an authorized worker checkpoint to its freshly proved body without
     * replacing a newer observation's emergency heading or reviving a revoked source. */
    public boolean rebaseForwardEmergency(MovementSync sync, Object publisher, Source original,
            Source live, WaypointFlightTarget route) {
        if (mode != Mode.LOCAL_SOLVER || owner != publisher || emergencySource == null
                || !currentSource(sync, original) || !matchesEmergencyCheckpoint(original, route)
                || !live.equals(snapshot(sync)) || !sameEmergencyCorridor(emergencyCorridor, route)) return false;
        emergencySource = live;
        return true;
    }

    /** Revoke an obsolete unpublished plan and fallback checkpoint. An already admitted
     * finite prefix remains subject to its own per-tick physical and corridor revalidation.
     */
    public void invalidateForwardEmergency(Object publisher) {
        if (mode != Mode.LOCAL_SOLVER || owner != publisher) return;
        if (pending != null && pending.owner() == publisher) pending = null;
        if (emergencySource != null) revokeEmergencyCheckpoint(emergencySource, emergencyCorridor);
        emergencySource = null;
        emergencyCorridor = null;
        emergencyCheckpointRevoked = true;
    }

    private boolean matchesEmergencyCheckpoint(Source source, WaypointFlightTarget route) {
        if (emergencyCheckpointRevoked || revokedEmergencyCheckpoint(source, route)) return false;
        if (emergencySource == null) return true;
        if (emergencySource == source)
            return sameEmergencyCorridor(emergencyCorridor, route);
        return emergencySource.tick() >= source.tick() && emergencyCorridor != null && route != null
                && emergencyCorridor.sameCorridor(route);
    }

    private void revokeEmergencyCheckpoint(Source source, WaypointFlightTarget corridor) {
        if (revokedEmergencyCheckpoint(source, corridor)) return;
        revokedEmergencyCheckpoints.add(new RevokedEmergencyCheckpoint(source,
                corridor == null ? null : corridor.corridorSnapshot()));
    }

    private boolean revokedEmergencyCheckpoint(Source source, WaypointFlightTarget corridor) {
        return revokedEmergencyCheckpoints.stream().anyMatch(checkpoint -> checkpoint.source() == source
                && sameEmergencyCorridor(checkpoint.corridor(), corridor));
    }

    private static boolean sameEmergencyCorridor(WaypointFlightTarget first,
            WaypointFlightTarget second) {
        return first == null ? second == null : first.sameCorridor(second);
    }

    private void pruneRevokedEmergencyCheckpoints(MovementSync sync) {
        revokedEmergencyCheckpoints.removeIf(checkpoint -> {
            long age = sync.physicsTickVersion() - checkpoint.source().tick();
            return checkpoint.source().epoch() != epoch
                    || checkpoint.source().generation() != sync.getNavigationGeneration()
                    || age < 0 || age > 8;
        });
    }

    private boolean currentSource(MovementSync sync, Source source) {
        long age = sync.physicsTickVersion() - source.tick();
        return source.epoch() == epoch && source.generation() == sync.getNavigationGeneration()
                && age >= 0 && age <= 8;
    }

    /** Diagnostic activation token, not an admission capability or controller ownership lock. */
    public record OwnershipTrace(long activation, long sourceTick, long sourceGeneration,
                                 long admittedAt, int cursor, int remainingExecutableTicks,
                                 boolean recovering, boolean active) {}
    private long traceActivation;

    /** Caller holds the same MovementSync monitor as consume; takes no other locks. */
    public OwnershipTrace ownershipTrace(long physicsTick) {
        if (active == null)
            return new OwnershipTrace(traceActivation, -1, -1, -1, -1, 0, false, false);
        int liveCursor = (int) Math.max(0, physicsTick - admittedAt);
        int remaining = Math.max(0, active.steps().size() - liveCursor);
        if (!recovering) remaining += active.recovery().size();
        return new OwnershipTrace(traceActivation, active.source().tick(), active.source().generation(),
                admittedAt, liveCursor, remaining, recovering, true);
    }

    /** Diagnostic snapshot at the last consumed cursor. */
    public OwnershipTrace ownershipTrace() {
        return ownershipTrace(active == null ? 0 : admittedAt + Math.max(0, cursor));
    }

    public static Source snapshot(MovementSync sync) {
        synchronized (sync) {
            return new Source(sync.physicsTickVersion(), sync.getNavigationGeneration(),
                    new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), sync.fallDistance.get(), PlayerPose.GLIDING), sync.flightControlEpoch());
        }
    }

    /** Immutable controls and corridor copy; no planner references cross into the physics monitor. */
    public static Envelope prepare(MovementSync sync, Object owner, Source source,
            ElytraFlightController.Decision selected, WaypointFlightTarget route, Runnable accepted) {
        var corridor = route == null ? null : route.corridorSnapshot();
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

    public void submit(Envelope envelope) {
        if (envelope.source().epoch() != epoch || consumed.contains(envelope)
                || owner != null && owner != envelope.owner()) return;
        pending = envelope;
    }

    /** Called under MovementSync, atomically with normal publication and consumption. */
    public void submitFallback(Envelope envelope) {
        if (pending != null) return;
        submit(envelope);
    }
    public void revoke(Object revoked) {
        if (owner == revoked || pending != null && pending.owner() == revoked
                || active != null && active.owner() == revoked) {
            clear();
            owner = null;
            mode = null;
        }
    }
    public void clear() { pending = null; active = null; batch = null; owner = null; mode = null; emergencySource = null; emergencyCorridor = null; emergencyCheckpointRevoked = false; revokedEmergencyCheckpoints.clear(); epoch++; }

    /** Terminal failure or an explicit landing handoff ends this physical fallback episode. */
    public void retireFallback(MovementSync sync) {
        if (fallbackActive) {
            sync.yaw.set(fallbackPriorYaw);
            sync.pitch.set(fallbackPriorPitch);
        }
        fallbackAnchor = null;
        fallbackTicks = 0;
        fallbackRecoveryTicks = 0;
        fallbackRecoveryPhysicsTick = Long.MIN_VALUE;
        fallbackAttitudePending = false;
        fallbackRequested = false;
        fallbackActive = false;
        if (failure != null && failure.responsibleOwner().equals("physics hazard owner")) failure = null;
    }

    /** Called under MovementSync only. Never takes navigation/controller locks or invokes search. */
    public void consume(MovementSync sync) {
        appliedThisTick = false;
        // Once a checked low/slow rescue owns the body, do not first accept a prefix
        // (or its action callback) that this same boundary would immediately suppress.
        if (fallbackActive) {
            hardFallback(sync);
            if (fallbackActive) return;
        }
        try { consumeOwned(sync); }
        finally { if (!appliedThisTick) hardFallback(sync); }
    }

    private boolean appliedThisTick;

    private void consumeOwned(MovementSync sync) {
        if (!sync.isGliding()) {
            clear();
            return;
        }
        if (pending != null && pending.witness() != null) {
            // Retire the outward emergency heading together with its old return episode.
            emergencySource = null;
            emergencyCorridor = null;
            var candidate = pending;
            pending = null;
            candidate = rebaseReturn(sync, candidate);
            if (candidate.source().epoch() == epoch && consumed.add(candidate)
                    && candidate.source().generation() == sync.getNavigationGeneration()
                    && candidate.source().tick() == sync.physicsTickVersion()
                    && returnMatches(sync, candidate, 0)) {
                active = candidate;
                admittedAt = sync.physicsTickVersion();
                traceActivation++;
                failure = null;
            } else {
                active = null;
                failure = new Failure(sync.physicsTickVersion(), sync.getNavigationGeneration(), epoch,
                        "Return source checkpoint expired", owner == null ? "return" : owner.getClass().getName());
            }
        }
        if (active != null && active.witness() != null) {
            int index = (int) (sync.physicsTickVersion() - admittedAt);
            if (!returnMatches(sync, active, index)) {
                active = null;
                failure = new Failure(sync.physicsTickVersion(), sync.getNavigationGeneration(), epoch,
                        "Return physical/resource checkpoint diverged", owner == null ? "return" : owner.getClass().getName());
                return;
            }
            var control = active.steps().get(index).control();
            sync.setYawToward(control.yaw());
            sync.pitch.set(control.pitch());
            if (control.firework() && lastReturnActionTick != sync.physicsTickVersion()) {
                lastReturnActionTick = sync.physicsTickVersion();
                var action = active.witness().action();
                sync.acceptFlightAction(() -> action.accept(control));
            }
            cursor = index;
            appliedThisTick = true;
            return;
        }
        if (pending != null) {
            var candidate = pending;
            pending = null;
            boolean fresh = candidate.source().epoch() == epoch && consumed.add(candidate);
            long age = sync.physicsTickVersion() - candidate.source().tick();
            boolean rocketSafe = !candidate.steps().isEmpty()
                    && (!candidate.steps().get(0).control().firework() || poweredSafe(sync, candidate));
            boolean controlSafe = fresh && candidate.source().generation() == sync.getNavigationGeneration()
                    && age >= 0 && age <= 8 && !candidate.steps().isEmpty() && revalidate(sync, candidate, 0);
            boolean horizonSafe = controlSafe && rocketSafe && preservesExecutableHorizon(sync, candidate);
            if (Boolean.getBoolean("traceAdmission"))
                System.err.println("PREFIX_GATE tick=" + sync.physicsTickVersion()
                        + " source=" + candidate.source().tick() + " rocket="
                        + (!candidate.steps().isEmpty() && candidate.steps().get(0).control().firework())
                        + " control=" + controlSafe + " rocketSafe=" + rocketSafe + " horizon=" + horizonSafe
                        + " count=" + candidate.steps().size() + "/" + candidate.recovery().size());
            if (horizonSafe) {
                active = candidate;
                failure = null;
                traceActivation++;
                cursor = 0;
                recovering = false;
                admittedAt = sync.physicsTickVersion();
                if (active.rejectedRocketTermination() != null)
                    active.rejectedRocketTermination().requestedAt = admittedAt;
                batch = TickControlBatch.admitted(active, admittedAt);
                // Actions are queued only after every admission gate has accepted the episode.
                sync.acceptFlightAction(candidate.accepted());
            } else if (fresh && active == null && candidate.source().generation() == sync.getNavigationGeneration()
                    && age >= 0 && age <= 8 && !candidate.recovery().isEmpty()) {
                var reserve = new Envelope(candidate.owner(), candidate.source(), candidate.recovery(),
                        candidate.corridor(), () -> {}, List.of(), candidate.handoff(), null,
                        candidate.requireAirborne(), null, null, candidate.preserveFallMargin());
                if (revalidate(sync, reserve, 0)) {
                    active = reserve;
                    traceActivation++;
                    cursor = 0;
                    admittedAt = sync.physicsTickVersion();
                    batch = TickControlBatch.admitted(active, admittedAt);
                    recovering = true;
                } else abandon(sync);
            } else if (fresh && active == null && candidate.source().generation() == sync.getNavigationGeneration()
                    && age >= 0 && age <= 8) {
                abandon(sync);
            }
        }
        if (active == null) {
            if (mode == Mode.LOCAL_SOLVER) abandon(sync);
            return;
        }
        if (active.source().generation() != sync.getNavigationGeneration() || !sync.isGliding()) {
            if (mode == Mode.LOCAL_SOLVER) abandon(sync);
            else clear();
            return;
        }
        cursor = (int) (sync.physicsTickVersion() - admittedAt);
        if (batch == null || !batch.authorizes(cursor, sync.physicsTickVersion()) || !revalidate(sync, active, cursor)) {
            // Never silently keep an expired navigation command. Normal airborne recovery
            // owns the next planning tick; do not fabricate touchdown or disable gliding.
            var rejected = active.rejectedRocketTermination();
            if (rejectionObserved(sync, rejected)) {
                var handOver = new Envelope(active.owner(), active.source(), rejected.handOver,
                        active.corridor(), () -> {}, List.of(), List.of(), null, true, null, null,
                        active.preserveFallMargin());
                if (!revalidate(sync, handOver, 0)) {
                    abandon(sync);
                    return;
                }
                active = handOver;
                traceActivation++;
                admittedAt = sync.physicsTickVersion();
                batch = TickControlBatch.admitted(active, admittedAt);
                cursor = 0;
                recovering = true;
            } else if (!recovering && !active.recovery().isEmpty()) {
                var reserve = new Envelope(active.owner(), active.source(), active.recovery(),
                        active.corridor(), () -> {}, List.of(), active.handoff(), null,
                        active.requireAirborne(), null, active.rejectedRocketTermination(),
                        active.preserveFallMargin());
                if (revalidate(sync, reserve, 0)) {
                    active = reserve;
                    traceActivation++;
                    admittedAt = sync.physicsTickVersion();
                    batch = TickControlBatch.admitted(active, admittedAt);
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
        appliedThisTick = true;
    }

    /** A bounded physical owner, not a route or a promise of reaching safe ground.
     * Every absent/rejected control is assessed at the input boundary, including epoch handoffs.
     * Only loaded, collision-free unpowered AND powered sweeps authorize a request.
     */
    // An owner/epoch handoff is not physical progress and must not replenish this budget.
    private org.joml.Vector3d fallbackAnchor;
    private int fallbackTicks;
    private int fallbackRecoveryTicks;
    private long fallbackRecoveryPhysicsTick = Long.MIN_VALUE;
    private boolean fallbackAttitudePending;
    private float fallbackYaw;
    private boolean fallbackRequested;
    private boolean fallbackActive;
    private float fallbackPriorYaw;
    private float fallbackPriorPitch;

    private void releaseFallback(MovementSync sync, String reason, double height) {
        fallbackRecoveryTicks = 0;
        fallbackRecoveryPhysicsTick = Long.MIN_VALUE;
        fallbackAttitudePending = false;
        if (!fallbackActive) return;
        MovementSync.getLogger().info("Flight fallback release: reason={} pos={} height={} owner={}",
                reason, sync.position.get(), height, owner == null ? "none" : owner.getClass().getName());
        fallbackActive = false;
        if (failure != null && failure.responsibleOwner().equals("physics hazard owner")) failure = null;
    }

    private void hardFallback(MovementSync sync) {
        if (sync.requiresNaturalDescent()) {
            releaseFallback(sync, "terminal-navigation-failure", Double.NaN);
            fallbackAnchor = null;
            return;
        }
        if (!sync.isGliding() || sync.onGround.get()) {
            releaseFallback(sync, "not-airborne", Double.NaN);
            fallbackAnchor = null;
            return;
        }
        // Movement-owned DIRECT/TRANSITION controls include independent landing, which
        // does not necessarily publish an envelope on every motion tick.
        if (owner instanceof xin.bbtt.movements.ElytraFlyMovement movement
                && movement.ownsIndependentLandingControl()) {
            releaseFallback(sync, "movement-landing-owner", Double.NaN);
            return;
        }
        if (mode == Mode.LOCAL_SOLVER && emergencySource != null
                && emergencySource.epoch() == epoch
                && emergencySource.generation() == sync.getNavigationGeneration()
                && sync.hasAttachedFirework() && failure != null
                && failure.reason().startsWith("UNPROVEN_EMERGENCY")) {
            // A rejected powered route may not be replaced by an unrelated climb
            // heading merely because the generic low/slow sweep has a shorter horizon.
            releaseFallback(sync, "registered-powered-emergency-unproven", Double.NaN);
            return;
        }
        var state = snapshot(sync).state();
        var velocity = state.velocity();
        if (!state.position().isFinite() || !velocity.isFinite() || !Float.isFinite(sync.yaw.get())) {
            releaseFallback(sync, "invalid-physical-state", Double.NaN);
            return;
        }
        var hazard = fallbackHazard(sync, state);
        if (hazard.status() == Observation.UNKNOWN) {
            fallbackRecoveryTicks = 0;
            fallbackRecoveryPhysicsTick = Long.MIN_VALUE;
            holdFallbackAttitude(sync, Double.NaN);
            return;
        }
        if (hazard.status() == Observation.CLEAR) {
            releaseFallback(sync, "outside-low-slow-hazard-band", Double.NaN);
            return;
        }
        double height = hazard.height();
        boolean recoveringNow = sync.hasAttachedFirework() && velocity.y > .05
                || velocity.y > .05 && Math.hypot(velocity.x, velocity.z) >= .8;
        if (recoveringNow) {
            // Count committed physical samples, not repeated calls to the shared input boundary.
            long physicalTick = sync.physicsTickVersion();
            if (!fallbackActive) return;
            if (fallbackRecoveryPhysicsTick != physicalTick) {
                if (fallbackRecoveryPhysicsTick != Long.MIN_VALUE
                        && physicalTick != fallbackRecoveryPhysicsTick + 1) fallbackRecoveryTicks = 0;
                fallbackRecoveryPhysicsTick = physicalTick;
                if (++fallbackRecoveryTicks >= 4) {
                    releaseFallback(sync, "sustained-recovery", height);
                    return;
                }
            }
        } else {
            fallbackRecoveryTicks = 0;
            fallbackRecoveryPhysicsTick = Long.MIN_VALUE;
        }
        var position = state.position();
        // Corrections, including alternating distant points, are not proven progress.
        // Only ending airborne flight renews this emergency budget.
        if (fallbackAnchor == null) {
            fallbackAnchor = position;
            fallbackTicks = 0;
            fallbackYaw = sync.yaw.get();
            fallbackRequested = false;
            fallbackAttitudePending = false;
        }
        if (fallbackTicks >= 24) {
            releaseFallback(sync, "stationary-episode-budget", height);
            return;
        }
        float yaw = fallbackYaw;
        float pitch = -55f;
        SweepStatus[] checked = {SweepStatus.BLOCKED, SweepStatus.BLOCKED};
        sync.getWorld().readTerrain(() -> {
            // Metadata may expire or arrive after this physical snapshot. Both futures
            // must be safe before fallback may steer or request another rocket.
            checked[0] = fallbackSweep(sync, state, yaw, pitch, false);
            checked[1] = checked[0] == SweepStatus.SAFE
                    ? fallbackSweep(sync, state, yaw, pitch, true) : checked[0];
            return checked[0] == SweepStatus.SAFE;
        });
        if (checked[0] == SweepStatus.UNKNOWN) {
            fallbackRecoveryTicks = 0;
            fallbackRecoveryPhysicsTick = Long.MIN_VALUE;
            holdFallbackAttitude(sync, height);
            return;
        }
        if (checked[0] == SweepStatus.BLOCKED) {
            releaseFallback(sync, "blocked-climb-sweep", height);
            return;
        }
        if (!fallbackActive) {
            MovementSync.getLogger().info(
                    "Flight fallback takeover: reason=low-slow-hazard pos={} height={} owner={}",
                    position, height, owner == null ? "none" : owner.getClass().getName());
            fallbackPriorYaw = sync.yaw.get();
            fallbackPriorPitch = sync.pitch.get();
        }
        fallbackActive = true;
        countFallbackAttitudeTick(sync);
        failure = new Failure(sync.physicsTickVersion(), sync.getNavigationGeneration(), epoch,
                "HARD_FALLBACK: bounded checked hazard climb", "physics hazard owner");
        sync.setYawToward(yaw);
        sync.pitch.set(pitch);
        if (checked[1] == SweepStatus.SAFE && !fallbackRequested && !sync.hasAttachedFirework() && sync.fireworkRequestReady()
                && sync.countHotbarFireworks() > 0) {
            fallbackRequested = true;
            sync.acceptFlightAction(() -> sync.useFirework());
        }
    }

    private void holdFallbackAttitude(MovementSync sync, double height) {
        if (!fallbackActive) return;
        if (fallbackTicks >= 24) {
            releaseFallback(sync, "stationary-episode-budget", height);
            return;
        }
        sync.setYawToward(fallbackYaw);
        sync.pitch.set(-55f);
        countFallbackAttitudeTick(sync);
    }

    private void countFallbackAttitudeTick(MovementSync sync) {
        fallbackAttitudePending = true;
    }

    /** Count only an attitude that actually reached a committed physics result. */
    public void completePhysicsStep() {
        if (!fallbackAttitudePending) return;
        fallbackAttitudePending = false;
        fallbackTicks++;
    }

    /** Only a nearby known hazardous surface (or the void boundary) is an emergency.
     * A distant hot column and accumulated fall distance are not climb permissions.
     */
    private enum Observation { HAZARD, CLEAR, UNKNOWN }
    private record HazardObservation(Observation status, double height) {}
    private enum SweepStatus { SAFE, BLOCKED, UNKNOWN }

    private static HazardObservation fallbackHazard(MovementSync sync, PhysicsState state) {
        var world = sync.getWorld();
        var feet = state.position();
        final double[] height = {Double.NaN};
        final boolean[] known = {true};
        world.readTerrain(() -> {
            double radius = PlayerPose.HALF_WIDTH;
            for (int x = (int) Math.floor(feet.x - radius); x <= Math.floor(feet.x + radius); x++) {
                for (int z = (int) Math.floor(feet.z - radius); z <= Math.floor(feet.z + radius); z++) {
                    if (!world.chunkLoaded(x >> 4, z >> 4)) {
                        known[0] = false;
                        return false;
                    }
                    boolean supported = false;
                    for (int y = (int) Math.floor(feet.y); y >= Math.max(
                            xin.bbtt.world.World.getMinWorldY(), (int) Math.ceil(feet.y - 12)); y--) {
                        var cell = new org.joml.Vector3d(x + .5, y, z + .5);
                        if (!FlightHazards.safeLanding(world, new org.joml.Vector3d(cell).add(0, 1, 0))) {
                            height[0] = feet.y - (y + 1);
                            supported = true;
                            break;
                        }
                        if (world.isSolid(cell)) { supported = true; break; }
                    }
                    if (!supported && feet.y <= xin.bbtt.world.World.getMinWorldY() + 12)
                        height[0] = feet.y - xin.bbtt.world.World.getMinWorldY();
                }
            }
            return true;
        });
        if (!known[0]) return new HazardObservation(Observation.UNKNOWN, Double.NaN);
        return Double.isFinite(height[0])
                ? new HazardObservation(Observation.HAZARD, height[0])
                : new HazardObservation(Observation.CLEAR, Double.NaN);
    }

    private static SweepStatus fallbackSweep(MovementSync sync, PhysicsState state,
            float yaw, float pitch, boolean powered) {
        return fallbackSweep(sync, state, yaw, pitch, powered, null);
    }

    private static SweepStatus fallbackSweep(MovementSync sync, PhysicsState state,
            float yaw, float pitch, boolean powered, WaypointFlightTarget corridor) {
        // Six unpowered delay ticks and twelve powered ticks, no terminal search.
        for (int i = 0; i < (powered ? 12 : 6); i++) {
            var next = new GlidingPhysics(yaw, pitch, powered, PlayerInput.NONE,
                    sync.getMovementParameters()).step(sync.getWorld(), state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            // Thrust is applied after travel and masks a ceiling-clipped zero Y velocity.
            var travel = powered ? new GlidingPhysics(yaw, pitch, false, PlayerInput.NONE,
                    sync.getMovementParameters()).step(sync.getWorld(), state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height()) : next;
            if (!FlightSafety.knownStep(sync.getWorld(), state, next)) {
                var position = next.position();
                return position.y < xin.bbtt.world.World.getMinWorldY()
                        || position.y + PlayerPose.GLIDING.height() > xin.bbtt.world.World.getMaxWorldY()
                        ? SweepStatus.BLOCKED : SweepStatus.UNKNOWN;
            }
            if (next.onGround() || travel.velocity().y == 0 || !safe(sync, state, next, corridor))
                return SweepStatus.BLOCKED;
            state = new PhysicsState(next.position(), next.velocity(), next.onGround(), next.fallDistance(), next.pose());
        }
        return SweepStatus.SAFE;
    }

    private Envelope rebaseReturn(MovementSync sync, Envelope candidate) {
        long age = sync.physicsTickVersion() - candidate.source().tick();
        var plan = candidate.witness().plan();
        // An aged active attachment or pending request cannot be shifted in time.
        // Only a quiescent, unused resource clock can restart its fixed controls.
        if (age <= 0 || age > 8 || candidate.source().epoch() != epoch
                || candidate.source().generation() != sync.getNavigationGeneration()
                || !plan.proven() || sync.hasAttachedFirework()
                || !plan.steps().get(0).resource().useRequested()
                || !java.util.Arrays.equals(candidate.witness().inventory(),
                        sync.inventoryManager.getHotbarFireworkFlightDurations())) return candidate;
        var current = snapshot(sync);
        final KnownReturnPlan.Result[] replay = {null};
        sync.getWorld().readTerrain(() -> {
            replay[0] = KnownReturnPlan.replay(sync.getWorld(), current.state(),
                    sync.getMovementParameters(), plan, sync.yaw.get());
            return replay[0].proven();
        });
        if (!replay[0].proven() || !consumed.add(candidate)) return candidate;
        return knownReturn(candidate.owner(), current, replay[0],
                candidate.witness().inventory(), candidate.witness().action());
    }

    private boolean returnMatches(MovementSync sync, Envelope envelope, int index) {
        if (index < 0 || index >= envelope.steps().size() || envelope.source().epoch() != epoch
                || envelope.source().generation() != sync.getNavigationGeneration()) return false;
        var witness = envelope.witness();
        var expected = witness.plan().steps().get(index);
        var before = expected.before();
        if (sync.position.get().distance(before.position()) > 1.0e-5
                || sync.velocity.get().distance(before.velocity()) > 1.0e-5
                || sync.onGround.get() != before.onGround()
                || Math.abs(sync.fallDistance.get() - before.fallDistance()) > 1.0e-5) return false;
        int used = index == 0 ? 0 : witness.plan().steps().get(index - 1).resource().checkpoint().consumed();
        if (!java.util.Arrays.equals(sync.inventoryManager.getHotbarFireworkFlightDurations(),
                java.util.Arrays.copyOfRange(witness.inventory(), used, witness.inventory().length))) return false;
        // A request is not an attachment. A missing/delayed or longer-than-preview
        // attachment invalidates the episode before any subsequent action is sent.
        if (!expected.resource().useRequested()
                && sync.hasAttachedFirework() != expected.resource().powered()) return false;
        if (expected.resource().useRequested() && sync.hasAttachedFirework()) return false;
        float executionYaw = MovementSync.nearestEquivalentYaw(sync.yaw.get(), expected.control().yaw());
        var result = new GlidingPhysics(executionYaw, expected.control().pitch(),
                expected.resource().powered(), PlayerInput.NONE, sync.getMovementParameters())
                .step(sync.getWorld(), before, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        return result.fallDistance() < sync.getMovementParameters().safeFallDistance()
                && FlightSafety.knownStep(sync.getWorld(), before, result)
                && safe(sync, before, result, null)
                && result.position().distance(expected.after().position()) <= 1.0e-5
                && result.velocity().distance(expected.after().velocity()) <= 1.0e-5;
    }

    private void abandon(MovementSync sync) {
        failure = new Failure(sync.physicsTickVersion(), sync.getNavigationGeneration(), epoch,
                "UNPROVEN_EMERGENCY: prepared continuation exhausted or failed revalidation",
                owner == null ? "legacy movement planner" : owner.getClass().getName());
        active = null;
        batch = null;
        if (mode == Mode.LOCAL_SOLVER && emergencySource != null && emergencyCorridor != null
                && emergencySource.epoch() == epoch
                && emergencySource.generation() == sync.getNavigationGeneration()) {
            // Registration is not permission to override normal descent or climb
            // through a roof. Legacy shallow recovery needs an actual safe sweep.
            var state = snapshot(sync).state();
            boolean checked = sync.getWorld().readTerrain(() ->
                    (state.fallDistance() <= 1
                        || fallbackHazard(sync, state).status() == Observation.HAZARD)
                    // Attachment may begin or end after this snapshot; both futures must be safe.
                    && fallbackSweep(sync, state, emergencyYaw, -20f,
                            false, emergencyCorridor) == SweepStatus.SAFE
                    && fallbackSweep(sync, state, emergencyYaw, -20f,
                            true, emergencyCorridor) == SweepStatus.SAFE);
            if (checked) {
                sync.setYawToward(emergencyYaw);
                sync.pitch.set(-20f);
            }
        }
        // Return responsibility to the existing movement planner. No fabricated descent,
        // no callback/search under this monitor, and no claim that residual motion is safe.
    }

    private static boolean poweredSafe(MovementSync sync, Envelope envelope) {
        var lifetime = envelope.rocketLifetime();
        // Legacy envelopes prepare an unpowered terminal continuation, not a full
        // rocket-lifetime episode. Their powered check covers only the replay window.
        if (lifetime == null) return rocketBranchSafe(sync, envelope, proofWindow(envelope, 0),
                Integer.MAX_VALUE);
        if (sync.hasAttachedFirework()) return false;
        int[] inventory = envelope.rejectedRocketTermination().publicationInventory;
        // No item at publication is the explicitly proved rejected-request outcome.
        // An item appearing later is not: it could attach an unproved lifetime.
        if (inventory != null && (!sync.fireworkRequestReady()
                || !java.util.Arrays.equals(inventory, sync.inventoryManager.getHotbarFireworkFlightDurations())
                || inventory.length > 0 && !lifetime.equals(FireworkSchedule.lifetimeBounds(inventory[0])))) return false;
        var controls = allControls(envelope);
        if (controls.size() <= lifetime.maximumPoweredTicks()) return false;
        if (!rejectedOutcomeSafe(sync, envelope, 0, 0)) return false;
        for (int poweredTicks = lifetime.minimumPoweredTicks();
                poweredTicks <= lifetime.maximumPoweredTicks(); poweredTicks++)
            if (!rocketBranchSafe(sync, envelope, controls, poweredTicks)) return false;
        return true;
    }

    private static boolean rocketBranchSafe(MovementSync sync, Envelope envelope,
            List<ElytraFlightController.Decision> controls, int poweredTicks) {
        return rocketBranchSafe(sync, envelope, controls, poweredTicks, liveState(sync));
    }

    private static boolean rocketBranchSafe(MovementSync sync, Envelope envelope,
            List<ElytraFlightController.Decision> controls, int poweredTicks,
            PhysicsState initial) {
        var state = initial;
        for (int tick = 0; tick < controls.size(); tick++) {
            var control = controls.get(tick);
            boolean powered = tick < poweredTicks;
            var result = new GlidingPhysics(control.yaw(), control.pitch(), powered, PlayerInput.NONE,
                    sync.getMovementParameters()).step(sync.getWorld(), state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (powered && result.onGround() || !safeTravel(sync, state, result, control,
                    envelope.corridor(), envelope.requireAirborne(), false, powered,
                    envelope.preserveFallMargin())) {
                return false;
            }
            state = FlightSafety.nextState(result);
            if (result.onGround()) return false;
        }
        return true;
    }

    /** A rejected request may leave navigation geometry only when the complete remaining
     * fixed episode closes on safe support. Re-entering the corridor does not erase departure. */
    private static boolean rejectedRocketBranchSafe(MovementSync sync, Envelope envelope,
            int index) {
        var controls = remainingControls(envelope, index);
        int executableTicks = envelope.steps().size() - index + envelope.recovery().size();
        var state = liveState(sync);
        boolean departedCorridor = false;
        for (int tick = 0; tick < controls.size(); tick++) {
            var control = controls.get(tick);
            var result = new GlidingPhysics(control.yaw(), control.pitch(), false, PlayerInput.NONE,
                    sync.getMovementParameters()).step(sync.getWorld(), state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            boolean insideCorridor = envelope.corridor() == null
                    || envelope.corridor().contains(state.position(), result.position());
            departedCorridor |= !insideCorridor;
            WaypointFlightTarget requiredCorridor = departedCorridor ? null : envelope.corridor();
            if (!safeTravel(sync, state, result, control, requiredCorridor, false, true, false,
                    envelope.preserveFallMargin()))
                return false;
            state = FlightSafety.nextState(result);
            if (result.onGround()) return true;
            // Handoff is proof-only. It may extend a corridor-bound safety window, but it
            // cannot promise the final off-corridor control needed to reach the ground.
            if (departedCorridor && tick + 1 >= executableTicks) return false;
        }
        return !departedCorridor;
    }

    /** Without thrust the remaining rocket controls must close safely, or, while rejection
     * is still unobserved, carry the body to the observation tick where the proved
     * unpowered hand-over can take over. */
    private static boolean rejectedOutcomeSafe(MovementSync sync, Envelope envelope, int index,
            long elapsed) {
        if (rejectedRocketBranchSafe(sync, envelope, index)) return true;
        var handOver = envelope.rejectedRocketTermination().handOver;
        if (handOver.isEmpty() || elapsed >= REJECTED_ROCKET_OBSERVATION_TICKS) return false;
        var controls = remainingControls(envelope, index);
        int untilObserved = REJECTED_ROCKET_OBSERVATION_TICKS - Math.toIntExact(elapsed);
        if (controls.size() < untilObserved) return false;
        var replay = new ArrayList<>(controls.subList(0, untilObserved));
        handOver.forEach(step -> replay.add(step.control()));
        var state = liveState(sync);
        for (var control : replay) {
            var result = new GlidingPhysics(control.yaw(), control.pitch(), false, PlayerInput.NONE,
                    sync.getMovementParameters()).step(sync.getWorld(), state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (!safeTravel(sync, state, result, control, envelope.corridor(), true, false, false,
                    envelope.preserveFallMargin())) return false;
            state = FlightSafety.nextState(result);
        }
        return true;
    }

    private static boolean rejectionObserved(MovementSync sync, RejectedRocketTermination lease) {
        return lease != null && !lease.handOver.isEmpty() && !lease.attachmentObserved
                && !sync.hasAttachedFirework() && lease.requestedAt != Long.MIN_VALUE
                && sync.physicsTickVersion() - lease.requestedAt >= REJECTED_ROCKET_OBSERVATION_TICKS;
    }

    private boolean requestedRocketSafe(MovementSync sync, Envelope envelope, int index,
            RejectedRocketTermination lease) {
        // Admission must prove the same unacknowledged state that will be consumed
        // immediately after dispatch; otherwise the callback can fire before revocation.
        long elapsed = lease.requestedAt == Long.MIN_VALUE ? 0
                : sync.physicsTickVersion() - lease.requestedAt;
        if (elapsed < 0) return false;
        if (lease.rejectionConfirmed)
            return rejectedRocketBranchSafe(sync, envelope, index);
        if (sync.hasAttachedFirework()) {
            lease.attachmentObserved = true;
            int minimumRemaining = Math.max(1,
                    lease.lifetime.minimumPoweredTicks() - Math.toIntExact(elapsed));
            int maximumRemaining = lease.lifetime.maximumPoweredTicks() - Math.toIntExact(elapsed);
            return poweredRemainingSafe(sync, envelope, remainingControls(envelope, index),
                    liveState(sync), minimumRemaining, maximumRemaining);
        }
        if (lease.attachmentObserved) return revalidateOrdinary(sync, envelope, index);
        if (elapsed >= lease.lifetime.maximumPoweredTicks()) {
            lease.rejectionConfirmed = true;
            return rejectedRocketBranchSafe(sync, envelope, index);
        }
        // Observed rejection keeps the rocket controls only while they stay safe without
        // thrust; otherwise the caller switches to the proved hand-over.
        if (rejectionObserved(sync, lease)) return rejectedRocketBranchSafe(sync, envelope, index);
        if (!rejectedOutcomeSafe(sync, envelope, index, elapsed)) return false;

        // The attachment snapshot is frozen for this physics tick. Prove the current
        // unpowered step, then every request-relative lifetime that could be observed next tick.
        var controls = remainingControls(envelope, index);
        if (controls.isEmpty()) return false;
        var control = controls.get(0);
        var initial = liveState(sync);
        var result = new GlidingPhysics(control.yaw(), control.pitch(), false, PlayerInput.NONE,
                sync.getMovementParameters()).step(sync.getWorld(), initial,
                        PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        long nextElapsed = elapsed + 1;
        if (nextElapsed >= lease.lifetime.maximumPoweredTicks()) return true;
        // Absence on the observation tick is rejection: the hand-over, already proved
        // above, owns that tick, not a later acknowledgement.
        if (!lease.handOver.isEmpty() && nextElapsed >= REJECTED_ROCKET_OBSERVATION_TICKS) return true;
        if (result.onGround() || controls.size() == 1) return false;
        int minimumRemaining = Math.max(1,
                lease.lifetime.minimumPoweredTicks() - Math.toIntExact(nextElapsed));
        int maximumRemaining = lease.lifetime.maximumPoweredTicks() - Math.toIntExact(nextElapsed);
        return poweredRemainingSafe(sync, envelope, controls.subList(1, controls.size()),
                FlightSafety.nextState(result), minimumRemaining, maximumRemaining);
    }

    private static boolean poweredRemainingSafe(MovementSync sync, Envelope envelope,
            List<ElytraFlightController.Decision> controls, PhysicsState initial,
            int minimumPoweredTicks, int maximumPoweredTicks) {
        if (minimumPoweredTicks <= 0 || maximumPoweredTicks < minimumPoweredTicks
                || controls.size() <= maximumPoweredTicks) return false;
        for (int poweredTicks = minimumPoweredTicks;
                poweredTicks <= maximumPoweredTicks; poweredTicks++)
            if (!rocketBranchSafe(sync, envelope, controls, poweredTicks, initial)) return false;
        return true;
    }

    private static PhysicsState liveState(MovementSync sync) {
        return new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(),
                sync.fallDistance.get(), PlayerPose.GLIDING);
    }

    private static List<ElytraFlightController.Decision> allControls(Envelope envelope) {
        return remainingControls(envelope, 0);
    }

    private static List<ElytraFlightController.Decision> remainingControls(Envelope envelope,
            int index) {
        var controls = new ArrayList<ElytraFlightController.Decision>(
                envelope.steps().size() - index + envelope.recovery().size()
                        + envelope.handoff().size());
        for (int step = index; step < envelope.steps().size(); step++)
            controls.add(envelope.steps().get(step).control());
        envelope.recovery().forEach(step -> controls.add(step.control()));
        envelope.handoff().forEach(step -> controls.add(step.control()));
        return controls;
    }

    private boolean preservesExecutableHorizon(MovementSync sync, Envelope candidate) {
        if (active == null || active.witness() != null
                || active.source().generation() != sync.getNavigationGeneration()) return true;
        int activeIndex = (int) (sync.physicsTickVersion() - admittedAt);
        int activeRemaining;
        boolean activeStillValid;
        if (activeIndex >= 0 && activeIndex < active.steps().size()) {
            activeRemaining = active.steps().size() - activeIndex + active.recovery().size();
            activeStillValid = revalidate(sync, active, activeIndex);
        } else if (!recovering && activeIndex == active.steps().size()
                && !active.recovery().isEmpty()) {
            var reserve = new Envelope(active.owner(), active.source(), active.recovery(),
                    active.corridor(), () -> {}, List.of(), active.handoff(), null,
                    active.requireAirborne(), null, active.rejectedRocketTermination(),
                    active.preserveFallMargin());
            activeRemaining = reserve.steps().size();
            activeStillValid = revalidate(sync, reserve, 0);
        } else {
            return true;
        }
        int candidateRemaining = candidate.steps().size() + candidate.recovery().size();
        return !activeStillValid || candidateRemaining >= activeRemaining
                || closesExecutableEpisode(sync, candidate);
    }

    private static boolean closesExecutableEpisode(MovementSync sync, Envelope envelope) {
        if (envelope.requireAirborne() || envelope.corridor() == null) return false;
        var state = new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(),
                sync.fallDistance.get(), PlayerPose.GLIDING);
        var controls = new ArrayList<ElytraFlightController.Decision>(
                envelope.steps().size() + envelope.recovery().size());
        envelope.steps().forEach(step -> controls.add(step.control()));
        envelope.recovery().forEach(step -> controls.add(step.control()));
        for (int index = 0; index < controls.size(); index++) {
            var control = controls.get(index);
            var result = new GlidingPhysics(control.yaw(), control.pitch(), sync.hasAttachedFirework(),
                    PlayerInput.NONE, sync.getMovementParameters()).step(sync.getWorld(), state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            boolean rejectedRocketLanding = index == 0 && control.firework();
            if (!safeTravel(sync, state, result, control, envelope.corridor(), false,
                    rejectedRocketLanding, sync.hasAttachedFirework(),
                    envelope.preserveFallMargin())) return false;
            state = FlightSafety.nextState(result);
            if (result.onGround()) return envelope.corridor().acceptsLanding(sync.getWorld(), state)
                    || rejectedRocketLanding && FlightHazards.safeLanding(sync.getWorld(), state.position());
        }
        return false;
    }

    private static List<ElytraFlightController.Decision> proofWindow(Envelope envelope, int index) {
        var controls = new ArrayList<ElytraFlightController.Decision>(6);
        appendControls(controls, envelope.steps(), index);
        appendControls(controls, envelope.recovery(), 0);
        appendControls(controls, envelope.handoff(), 0);
        return controls;
    }

    private static void appendControls(List<ElytraFlightController.Decision> controls,
            List<Step> steps, int index) {
        for (int i = index; i < steps.size() && controls.size() < 6; i++)
            controls.add(steps.get(i).control());
    }

    private boolean revalidate(MovementSync sync, Envelope envelope, int index) {
        if (index < 0 || index >= envelope.steps().size()) return false;
        var rejectedRocketTermination = envelope.rejectedRocketTermination();
        if (rejectedRocketTermination != null)
            return requestedRocketSafe(sync, envelope, index, rejectedRocketTermination);
        return revalidateOrdinary(sync, envelope, index);
    }

    private boolean revalidateOrdinary(MovementSync sync, Envelope envelope, int index) {
        var controls = proofWindow(envelope, index);
        int maximumPoweredTicks = sync.hasAttachedFirework() ? controls.size() : 0;
        PhysicsState state = null;
        for (int poweredTicks = 0; poweredTicks <= maximumPoweredTicks; poweredTicks++) {
            state = new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(),
                    sync.fallDistance.get(), PlayerPose.GLIDING);
            for (int tick = 0; tick < controls.size(); tick++) {
                var control = controls.get(tick);
                boolean powered = tick < poweredTicks;
                var result = new GlidingPhysics(control.yaw(), control.pitch(), powered,
                        PlayerInput.NONE, sync.getMovementParameters()).step(sync.getWorld(), state,
                                PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                if (!safeTravel(sync, state, result, control, envelope.corridor(),
                        envelope.requireAirborne(), false, powered, envelope.preserveFallMargin())) {
                    return false;
                }
                state = FlightSafety.nextState(result);
                if (result.onGround()) break;
            }
        }
        // A short landing prefix must not spend the last recoverable descent tick.
        // Replay the fixed fallback attitude from its end, rather than assuming
        // that a one-tick-safe dive leaves a usable continuation. Cruise/Follow
        // prefixes retain their terrain-checked replay, not a landing-only fall budget.
        if (mode == Mode.LOCAL_SOLVER
                && envelope.steps().get(index).control().phase() == ElytraFlightController.Phase.LANDING
                && !state.onGround() && state.fallDistance() > 1
                && !checkedLanding(sync, state, envelope.steps().get(index).control(), envelope.corridor(),
                        envelope.preserveFallMargin())) {
            float yaw = envelope.steps().get(index).control().yaw();
            for (int i = 0; i < 6; i++) {
                var result = new GlidingPhysics(yaw, -55f, sync.hasAttachedFirework(),
                        PlayerInput.NONE, sync.getMovementParameters()).step(sync.getWorld(), state,
                                PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                if (result.fallDistance() >= sync.getMovementParameters().safeFallDistance()
                        || !safe(sync, state, result, envelope.corridor())) return false;
                state = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                        result.fallDistance(), result.pose());
                if (result.onGround()) break;
            }
        }
        return true;
    }

    /** A normal descent may prove safe touchdown instead of an unrelated climb.
     * A phase label or solid column alone is not a completion proof.
     */
    private static boolean checkedLanding(MovementSync sync, PhysicsState state,
            ElytraFlightController.Decision control, WaypointFlightTarget route) {
        return checkedLanding(sync, state, control, route, false);
    }

    private static boolean checkedLanding(MovementSync sync, PhysicsState state,
            ElytraFlightController.Decision control, WaypointFlightTarget route,
            boolean preserveFallMargin) {
        if (control.phase() != ElytraFlightController.Phase.LANDING || control.firework()
                || sync.hasAttachedFirework()) return false;
        for (int i = 0; i < 80; i++) {
            var next = new GlidingPhysics(control.yaw(), control.pitch(), false, PlayerInput.NONE,
                    sync.getMovementParameters()).step(sync.getWorld(), state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (preserveFallMargin && !(next.fallDistance() < sync.getMovementParameters().safeFallDistance()))
                return false;
            if (!FlightSafety.knownStep(sync.getWorld(), state, next)
                    || !safe(sync, state, next, route)) return false;
            if (next.onGround()) return (route == null
                        ? FlightHazards.safeLanding(sync.getWorld(), next.position())
                        : route.acceptsLanding(sync.getWorld(), new PhysicsState(next.position(), next.velocity(), true,
                                next.fallDistance(), next.pose())))
                    && new FlightSafety(sync.getMovementParameters(), DamageParameters.NONE, 20)
                            .collisionDamage(sync.getWorld(), state, next, control.yaw(), control.pitch(), false).raw() == 0;
            state = new PhysicsState(next.position(), next.velocity(), false, next.fallDistance(), next.pose());
        }
        return false;
    }

    /** Inspect pre-thrust travel from the same state: thrust can hide a ceiling-clipped Y. */
    private static boolean safeTravel(MovementSync sync, PhysicsState before, PhysicsResult after,
            ElytraFlightController.Decision control, WaypointFlightTarget corridor,
            boolean requireAirborne, boolean rejectedRocketLanding, boolean powered,
            boolean preserveFallMargin) {
        if (preserveFallMargin && !(after.fallDistance() < sync.getMovementParameters().safeFallDistance()))
            return false;
        if (!FlightSafety.knownStep(sync.getWorld(), before, after)) return false;
        if (requireAirborne && !rejectedRocketLanding
                && (before.onGround() || after.onGround())) return false;
        if (after.onGround()) {
            if (powered) return false;
            if (new FlightSafety(sync.getMovementParameters(), DamageParameters.NONE, 20)
                    .collisionDamage(sync.getWorld(), before, after, control.yaw(), control.pitch(), false)
                    .raw() != 0) return false;
            if (rejectedRocketLanding) {
                if (!FlightHazards.safeLanding(sync.getWorld(), after.position())) return false;
            } else if (corridor != null
                    && !corridor.acceptsLanding(sync.getWorld(), FlightSafety.nextState(after))) {
                return false;
            }
        }
        if (before.velocity().y > .003 && !after.onGround()) {
            var travel = new GlidingPhysics(control.yaw(), control.pitch(), false, PlayerInput.NONE,
                    sync.getMovementParameters()).step(sync.getWorld(), before,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (travel.velocity().y == 0) return false;
        }
        return safe(sync, before, after, corridor);
    }

    private static boolean safe(MovementSync sync, PhysicsState before, PhysicsResult after, WaypointFlightTarget corridor) {
        var p = after.position();
        return p.isFinite() && FlightSafety.knownStep(sync.getWorld(), before, after)
                && sync.getWorld().chunkLoaded(((int)Math.floor(p.x)) >> 4, ((int)Math.floor(p.z)) >> 4)
                && !after.collidedX() && !after.collidedZ()
                && FlightHazards.damage(sync.getWorld(), before, after, DamageParameters.NONE, true).raw() == 0
                && FlightHazards.damage(sync.getWorld(), before, after, DamageParameters.NONE, false).raw() == 0
                && (corridor == null || corridor.contains(before.position(), p));
    }
}
