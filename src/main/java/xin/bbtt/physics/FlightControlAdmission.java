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
    public record ReturnWitness(KnownReturnPlan.Result plan, int[] inventory,
                                java.util.function.Consumer<ElytraFlightController.Decision> action) {
        public ReturnWitness { inventory = inventory.clone(); }
        @Override public int[] inventory() { return inventory.clone(); }
    }
    public record Envelope(Object owner, Source source, List<Step> steps,
                           WaypointFlightTarget corridor, Runnable accepted, List<Step> recovery,
                           ReturnWitness witness, boolean requireAirborne) {
        public Envelope(Object owner, Source source, List<Step> steps, WaypointFlightTarget corridor,
                Runnable accepted, List<Step> recovery, ReturnWitness witness) {
            this(owner, source, steps, corridor, accepted, recovery, witness, false);
        }
        public Envelope(Object owner, Source source, List<Step> steps, WaypointFlightTarget corridor,
                Runnable accepted, List<Step> recovery) {
            this(owner, source, steps, corridor, accepted, recovery, null, false);
        }
        public Envelope { steps = List.copyOf(steps); recovery = List.copyOf(recovery); }
    }

    /** Entire event-time episode; resource actions are dispatched at their own checked tick. */
    public static Envelope knownReturn(Object owner, Source source, KnownReturnPlan.Result plan,
            int[] inventory, java.util.function.Consumer<ElytraFlightController.Decision> action) {
        return new Envelope(owner, source, plan.steps().stream()
                .map(step -> new Step(step.control(), step.before())).toList(), null, () -> {}, List.of(),
                new ReturnWitness(plan, inventory, action));
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

    /** Copy the solver's exact executable prefix and fixed reserve; never search for a terminal. */
    public static Envelope createPrefix(Object owner, Source source, List<Step> prefix,
            List<Step> recovery, WaypointFlightTarget route, Runnable accepted) {
        return createPrefix(owner, source, prefix, recovery, route, accepted, false);
    }

    /** Moving pursuit prefixes must remain airborne when replayed from a newer live state. */
    public static Envelope createPrefix(Object owner, Source source, List<Step> prefix,
            List<Step> recovery, WaypointFlightTarget route, Runnable accepted,
            boolean requireAirborne) {
        var corridor = route == null ? null : route.corridorSnapshot();
        if (recovery.stream().anyMatch(step -> step.control().firework()))
            throw new IllegalArgumentException("Fixed recovery cannot request rockets");
        if (!prefix.isEmpty() && recovery.stream().anyMatch(step -> step.control().pitch() > 0
                || Math.cos(Math.toRadians(step.control().yaw() - prefix.get(0).control().yaw())) <= 0))
            recovery = List.of(); // A targetless reserve is not permission for reverse descent.
        return new Envelope(owner, source, prefix, corridor, accepted, recovery, null, requireAirborne);
    }

    /** Caller holds sync. Invalid/stale publication never displaces the active prefix. */
    public boolean publishPrefix(MovementSync sync, Object publisher, Source source, List<Step> prefix,
            List<Step> recovery, WaypointFlightTarget route, Runnable accepted) {
        return publishPrefix(sync, publisher, source, prefix, recovery, route, accepted, false);
    }

    /** Caller holds sync. Invalid/stale publication never displaces the active prefix. */
    public boolean publishPrefix(MovementSync sync, Object publisher, Source source, List<Step> prefix,
            List<Step> recovery, WaypointFlightTarget route, Runnable accepted,
            boolean requireAirborne) {
        if (mode != Mode.LOCAL_SOLVER || owner != publisher || !currentSource(sync, source)
                || emergencySource != null && emergencySource != source) return false;
        submit(createPrefix(publisher, source, prefix, recovery, route, accepted, requireAirborne));
        return true;
    }

    public boolean registerForwardEmergency(MovementSync sync, Object publisher, Source source,
            float yaw) {
        return registerForwardEmergency(sync, publisher, source, yaw, null);
    }

    /** Caller holds sync. Heading and corridor are one immutable physical checkpoint. */
    public boolean registerForwardEmergency(MovementSync sync, Object publisher, Source source,
            float yaw, WaypointFlightTarget route) {
        if (mode != Mode.LOCAL_SOLVER || owner != publisher || !currentSource(sync, source)
                || !Float.isFinite(yaw)
                || emergencySource != null && source.tick() < emergencySource.tick()) return false;
        emergencySource = source;
        emergencyYaw = yaw;
        emergencyCorridor = route == null ? null : route.corridorSnapshot();
        return true;
    }

    /** A worker may refine only the exact checkpoint that launched it. */
    public boolean refineForwardEmergency(MovementSync sync, Object publisher, Source source,
            float yaw, WaypointFlightTarget route) {
        if (mode != Mode.LOCAL_SOLVER || owner != publisher || !currentSource(sync, source)
                || !Float.isFinite(yaw) || emergencySource != source || emergencyCorridor == null
                || route == null || !emergencyCorridor.sameCorridor(route)) return false;
        emergencyYaw = yaw;
        return true;
    }

    private boolean currentSource(MovementSync sync, Source source) {
        long age = sync.physicsTickVersion() - source.tick();
        return source.epoch() == epoch && source.generation() == sync.getNavigationGeneration()
                && age >= 0 && age <= 8;
    }

    /** Diagnostic activation token, not an admission capability or controller ownership lock. */
    public record OwnershipTrace(long activation, long sourceTick, long sourceGeneration,
                                 long admittedAt, int cursor, boolean recovering, boolean active) {}
    private long traceActivation;

    /** Caller holds the same MovementSync monitor as consume; takes no other locks. */
    public OwnershipTrace ownershipTrace() {
        return active == null
                ? new OwnershipTrace(traceActivation, -1, -1, -1, -1, false, false)
                : new OwnershipTrace(traceActivation, active.source().tick(), active.source().generation(),
                        admittedAt, cursor, recovering, true);
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
    public void revoke(Object revoked) {
        if (owner == revoked || pending != null && pending.owner() == revoked
                || active != null && active.owner() == revoked) {
            clear();
            owner = null;
            mode = null;
        }
    }
    public void clear() { pending = null; active = null; batch = null; owner = null; mode = null; emergencySource = null; emergencyCorridor = null; epoch++; }

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
            if (fresh && candidate.source().generation() == sync.getNavigationGeneration() && age >= 0 && age <= 8
                    && !candidate.steps().isEmpty() && revalidate(sync, candidate, 0)) {
                active = candidate;
                failure = null;
                traceActivation++;
                cursor = 0;
                recovering = false;
                admittedAt = sync.physicsTickVersion();
                batch = TickControlBatch.admitted(active, admittedAt);
                // Actions are queued only at acceptance, immediately before the packet/input batch.
                // Recheck the powered branch as well: rejecting thrust must reject its action,
                // not silently rely on the unpowered continuation proof.
                if (!candidate.steps().get(0).control().firework() || poweredSafe(sync, candidate))
                    sync.acceptFlightAction(candidate.accepted());
            } else if (fresh && active == null && candidate.source().generation() == sync.getNavigationGeneration()
                    && age >= 0 && age <= 8 && !candidate.recovery().isEmpty()) {
                var reserve = new Envelope(candidate.owner(), candidate.source(), candidate.recovery(),
                        candidate.corridor(), () -> {}, List.of(), null, candidate.requireAirborne());
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
            if (!recovering && !active.recovery().isEmpty()) {
                var reserve = new Envelope(active.owner(), active.source(), active.recovery(),
                        active.corridor(), () -> {}, List.of(), null, active.requireAirborne());
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
                    sync.getMovementParameters(), plan);
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
        var result = new GlidingPhysics(expected.control().yaw(), expected.control().pitch(),
                expected.resource().powered(), PlayerInput.NONE, sync.getMovementParameters())
                .step(sync.getWorld(), before, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
        return FlightSafety.knownStep(sync.getWorld(), before, result)
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
        var state = new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), sync.fallDistance.get(), PlayerPose.GLIDING);
        for (int i = 0; i < Math.min(6, envelope.steps().size()); i++) {
            var control = envelope.steps().get(i).control();
            var result = new GlidingPhysics(control.yaw(), control.pitch(), true, PlayerInput.NONE,
                    sync.getMovementParameters()).step(sync.getWorld(), state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (result.onGround() || !safeTravel(sync, state, result, control, envelope.corridor(),
                    envelope.requireAirborne())) return false;
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance(), result.pose());
        }
        return true;
    }

    private boolean revalidate(MovementSync sync, Envelope envelope, int index) {
        if (index < 0 || index >= envelope.steps().size()) return false;
        var state = new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), sync.fallDistance.get(), PlayerPose.GLIDING);
        // Fixed maximum of six physics steps, selected controls only, no candidate search.
        for (int i = index; i < Math.min(index + 6, envelope.steps().size()); i++) {
            var control = envelope.steps().get(i).control();
            var result = new GlidingPhysics(control.yaw(), control.pitch(), sync.hasAttachedFirework(),
                    PlayerInput.NONE, sync.getMovementParameters()).step(sync.getWorld(), state,
                    PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (!safeTravel(sync, state, result, control, envelope.corridor(),
                    envelope.requireAirborne())) return false;
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance(), result.pose());
            if (result.onGround()) break;
        }
        // A short landing prefix must not spend the last recoverable descent tick.
        // Replay the fixed fallback attitude from its end, rather than assuming
        // that a one-tick-safe dive leaves a usable continuation. Cruise/Follow
        // prefixes retain their terrain-checked replay, not a landing-only fall budget.
        if (mode == Mode.LOCAL_SOLVER
                && envelope.steps().get(index).control().phase() == ElytraFlightController.Phase.LANDING
                && !state.onGround() && state.fallDistance() > 1
                && !checkedLanding(sync, state, envelope.steps().get(index).control(), envelope.corridor())) {
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
        if (control.phase() != ElytraFlightController.Phase.LANDING || control.firework()
                || sync.hasAttachedFirework()) return false;
        for (int i = 0; i < 80; i++) {
            var next = new GlidingPhysics(control.yaw(), control.pitch(), false, PlayerInput.NONE,
                    sync.getMovementParameters()).step(sync.getWorld(), state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
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
            boolean requireAirborne) {
        if (requireAirborne && (before.onGround() || after.onGround())) return false;
        if (after.onGround() && (sync.hasAttachedFirework() || corridor != null
                && !corridor.acceptsLanding(sync.getWorld(), FlightSafety.nextState(after)))) return false;
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
        return p.isFinite() && sync.getWorld().chunkLoaded(((int)Math.floor(p.x)) >> 4, ((int)Math.floor(p.z)) >> 4)
                && !after.collidedX() && !after.collidedZ()
                && FlightHazards.damage(sync.getWorld(), before, after, DamageParameters.NONE, true).raw() == 0
                && FlightHazards.damage(sync.getWorld(), before, after, DamageParameters.NONE, false).raw() == 0
                && (corridor == null || corridor.contains(before.position(), p));
    }
}
