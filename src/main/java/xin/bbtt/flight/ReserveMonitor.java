package xin.bbtt.flight;

import java.util.Arrays;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.FlightControlAdmission;
import xin.bbtt.physics.HazardRecovery;
import xin.bbtt.physics.KnownLandingReserve;
import xin.bbtt.world.World;

/** Advisory actual-state reserve observer. Never owns, submits or applies flight controls. */
public final class ReserveMonitor {
    public static final int INTERVAL_TICKS = 10;
    public enum Status { SAFE, UNSAFE, UNKNOWN, STALE }

    /** SAFE means the selected observation policy succeeded: legacy recovery admission,
     * or a valid known-site estimate (including its return threshold). It does not
     * admit future controls. UNKNOWN and STALE forbid deeper exposure.
     */
    public record Report(Status status, long tick, long generation, long epoch,
                         int simulatedSteps, String reason, KnownLandingReserve.Result reserve) {
        public Report(Status status, long tick, long generation, long epoch, int steps, String reason) {
            this(status, tick, generation, epoch, steps, reason, null);
        }
    }

    private static final ExecutorService WORKER = Executors.newSingleThreadExecutor(r -> {
        Thread thread = new Thread(r, "flight-reserve");
        thread.setDaemon(true);
        return thread;
    });
    // Global admission gate: at most one running/submitted job, even across sessions.
    private static final AtomicBoolean BUSY = new AtomicBoolean();
    private java.util.concurrent.Executor executor = WORKER;

    /** Instance-scoped test scheduler; install before the first observation. */
    synchronized void setExecutorForTesting(java.util.concurrent.Executor executor) {
        if (observedSync != null) throw new IllegalStateException("Already observed");
        this.executor = Objects.requireNonNull(executor);
    }

    private MovementSync observedSync;
    private World observedWorld;
    private long generation, epoch, eventToken, revision;
    private HazardRecovery.Context observedContext;
    private boolean observedKnownReturn;
    private long submittedRevision = -1;
    private long lastTick = Long.MIN_VALUE;
    private Report latest = new Report(Status.UNKNOWN, -1, -1, -1, 0, "Not observed");
    private KnownLandingReserve.Result returnIntent;
    private Report returnIntentReport;
    private HazardRecovery.Context returnIntentResources;

    public synchronized Report returnIntentReport() { return returnIntent() == null ? null : returnIntentReport; }
    public synchronized HazardRecovery.Context returnIntentResources() {
        return returnIntent() == null ? null : returnIntentResources;
    }

    /** Distinguish exhausted fuel from a failed proof; neither permits deeper exposure. */
    public static String returnTrigger(KnownLandingReserve.Result reserve) {
        if (reserve == null) return "NO_OBSERVATION";
        if (reserve.status() != KnownLandingReserve.Status.VALID)
            return "RESERVE_" + reserve.status();
        return reserve.shouldReturn() ? "RESOURCE_THRESHOLD" : "POSITIVE_SURPLUS";
    }

    /** Retained return intent never grants permission to continue an expired prefix. */
    public synchronized KnownLandingReserve.Result returnIntent() {
        if (observedSync == null || observedSync.getWorld() != observedWorld
                || observedSync.getNavigationGeneration() != generation
                || observedSync.flightControlEpoch() != epoch) return null;
        return returnIntent;
    }

    private volatile CompletableFuture<Void> completion = CompletableFuture.completedFuture(null);

    /** Planner thread API, outside the MovementSync monitor. The token must change
     * immediately for route replacement, off-route transition or health change (and
     * any other caller-owned invalidation). Inventory, attachment and cooldown/boost
     * availability transitions are also detected here. Normal countdown aging does
     * not bypass the ten-physical-tick cadence.
     *
     * Returns true only when work was submitted. Busy requests are dropped, not queued;
     * an event remains eligible on the next observe call. Even a same-tick event can
     * request a new proof. Context resources and the Source state are immutable copies.
     */
    public boolean observe(MovementSync sync, FlightControlAdmission.Source source,
            long eventToken, HazardRecovery.Context context) {
        return observe(sync, source, eventToken, context, false);
    }

    /** Known-supported-site resource policy; shares cadence and invalidation with observe.
     * A SAFE report carries a valid reserve estimate, not necessarily positive surplus.
     */
    public boolean observeKnownReturn(MovementSync sync, FlightControlAdmission.Source source,
            long eventToken, HazardRecovery.Context context) {
        return observe(sync, source, eventToken, context, true);
    }

    private boolean observe(MovementSync sync, FlightControlAdmission.Source source,
            long eventToken, HazardRecovery.Context context, boolean knownReturn) {
        Objects.requireNonNull(sync);
        Objects.requireNonNull(source);
        Objects.requireNonNull(context);
        // Do not introduce physical-monitor -> observer lock ordering or start search there.
        if (Thread.holdsLock(sync)) return false;
        synchronized (this) {
            World world = sync.getWorld();
            if (!current(sync, world, source)) {
                stale("Source generation or epoch is obsolete");
                return false;
            }
            if (observedSync != sync || observedWorld != world || generation != source.generation()
                    || epoch != source.epoch() || this.eventToken != eventToken
                    || observedKnownReturn != knownReturn) returnIntent = null;
            boolean changed = observedSync != sync || observedWorld != world
                    || generation != source.generation() || epoch != source.epoch()
                    || this.eventToken != eventToken || observedKnownReturn != knownReturn
                    || resourcesChanged(observedContext, context);
            if (changed) {
                revision++;
                stale("Reserve inputs changed");
            }
            observedSync = sync;
            observedWorld = world;
            generation = source.generation();
            epoch = source.epoch();
            this.eventToken = eventToken;
            observedContext = context;
            observedKnownReturn = knownReturn;
            boolean due = submittedRevision != revision || lastTick == Long.MIN_VALUE
                    || source.tick() < lastTick || source.tick() - lastTick >= INTERVAL_TICKS;
            if (!due) return false;
            stale("Reserve refresh due");
            if (!BUSY.compareAndSet(false, true)) return false;
            var parameters = sync.getMovementParameters();
            long jobRevision = revision;
            lastTick = source.tick();
            submittedRevision = revision;
            var finished = new CompletableFuture<Void>();
            completion = finished;
            try {
                executor.execute(() -> {
                    var budget = new HazardRecovery.Budget(HazardRecovery.DECISION_STEP_BUDGET);
                    try {
                        // No MovementSync lock is held across terrain queries or search.
                        if (knownReturn) {
                            var result = KnownLandingReserve.assess(world, source.state(), parameters, context,
                                    DangerousEntryGate.knownSites(world, source.state(), context), 10, 1);
                            publish(sync, world, source, jobRevision, new Report(
                                    result.status() == KnownLandingReserve.Status.VALID ? Status.SAFE : Status.UNKNOWN,
                                    source.tick(), source.generation(), source.epoch(), 0,
                                    result.status().name(), result));
                        } else {
                            var result = HazardRecovery.assess(world, source.state(), parameters, context, budget);
                            publish(sync, world, source, jobRevision, new Report(
                                    result.admissible() ? Status.SAFE : Status.UNKNOWN,
                                    source.tick(), source.generation(), source.epoch(), budget.steps(), result.reason()));
                        }
                    } catch (RuntimeException error) {
                        publish(sync, world, source, jobRevision, new Report(Status.UNKNOWN,
                                source.tick(), source.generation(), source.epoch(), budget.steps(),
                                "Reserve observation failed: " + error.getClass().getSimpleName()));
                    } finally {
                        BUSY.set(false);
                        finished.complete(null);
                    }
                });
            } catch (RuntimeException error) {
                BUSY.set(false);
                submittedRevision = -1;
                finished.complete(null);
                throw error;
            }
            return true;
        }
    }

    /** Invalidates completed reports on world/generation/epoch changes even without
     * another observe call. Reports are immutable values, never control capabilities.
     */
    public synchronized Report latest() {
        if (observedSync != null && (observedSync.getWorld() != observedWorld
                || observedSync.getNavigationGeneration() != generation
                || observedSync.flightControlEpoch() != epoch)) stale("Reserve ownership changed");
        if (observedSync != null && latest.tick() >= 0
                && observedSync.physicsTickVersion() - latest.tick() >= INTERVAL_TICKS)
            stale("Reserve observation expired");
        return latest;
    }

    private synchronized void publish(MovementSync sync, World world,
            FlightControlAdmission.Source source, long jobRevision, Report report) {
        // An obsolete producer owns no publication rights. Invalidation already happened
        // on the observation/consumer path; never overwrite a newer report with its discard.
        if (revision != jobRevision || !current(sync, world, source)) return;
        if (report.reserve() != null && report.reserve().shouldReturn()) {
            returnIntent = report.reserve();
            returnIntentReport = report;
            returnIntentResources = observedContext;
        }
        latest = sync.physicsTickVersion() - source.tick() >= INTERVAL_TICKS
                ? new Report(Status.STALE, report.tick(), report.generation(), report.epoch(),
                        report.simulatedSteps(), "Reserve observation expired during search") : report;
    }

    /** Explicit exact-return diagnostic: report the accepted trajectory's actual
     * rocket consumption, not the conservative distance estimate. The periodic
     * observer calls KnownLandingReserve.assess directly and never invokes this helper. */
    static KnownLandingReserve.Result resolveReturnEstimate(World world,
            xin.bbtt.physics.PhysicsState state, xin.bbtt.physics.MovementParameters parameters,
            HazardRecovery.Context resources, KnownLandingReserve.Result estimate) {
        if (estimate.status() != KnownLandingReserve.Status.UNPROVEN || estimate.returnSite() == null)
            return estimate;
        var plan = xin.bbtt.physics.KnownReturnPlan.preview(world, state, parameters,
                resources, estimate.returnSite(), 10);
        if (!plan.proven()) return estimate;
        int required = plan.requiredRockets();
        return new KnownLandingReserve.Result(KnownLandingReserve.Status.VALID,
                estimate.returnSite(), required, estimate.safetyMarginRockets(),
                resources.rocketFlightDurations().length - required - estimate.safetyMarginRockets(),
                true, true);
    }

    private static boolean current(MovementSync sync, World world, FlightControlAdmission.Source source) {
        return sync.getWorld() == world && sync.getNavigationGeneration() == source.generation()
                && sync.flightControlEpoch() == source.epoch();
    }

    private static boolean resourcesChanged(HazardRecovery.Context before, HazardRecovery.Context after) {
        return before == null || !Arrays.equals(before.rocketFlightDurations(), after.rocketFlightDurations())
                || before.attached() != after.attached()
                || (before.cooldownTicks() == 0) != (after.cooldownTicks() == 0)
                || (before.guaranteedBoostTicks() > 0) != (after.guaranteedBoostTicks() > 0)
                || before.memory() != after.memory();
    }

    private void stale(String reason) {
        latest = new Report(Status.STALE, latest.tick(), latest.generation(), latest.epoch(),
                latest.simulatedSteps(), reason);
    }

    /** Test/diagnostic barrier for the most recently accepted observation, including discards. */
    boolean awaitIdle(long timeout, TimeUnit unit) throws InterruptedException,
            java.util.concurrent.ExecutionException, java.util.concurrent.TimeoutException {
        completion.get(timeout, unit);
        return true;
    }
}
