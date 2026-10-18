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
import xin.bbtt.world.World;

/** Advisory actual-state reserve observer. Never owns, submits or applies flight controls. */
public final class ReserveMonitor {
    public static final int INTERVAL_TICKS = 10;
    public enum Status { SAFE, UNSAFE, UNKNOWN, STALE }

    /** SAFE is HazardRecovery's admissible result, not admission of a future control.
     * UNKNOWN includes exhausted search and unknown terrain: failure to prove a landing
     * is not proof of impossibility. UNSAFE is reserved for a definitive negative proof;
     * HazardRecovery currently exposes no such result.
     */
    public record Report(Status status, long tick, long generation, long epoch,
                         int simulatedSteps, String reason) {}

    private static final ExecutorService WORKER = Executors.newSingleThreadExecutor(r -> {
        Thread thread = new Thread(r, "flight-reserve");
        thread.setDaemon(true);
        return thread;
    });
    // Global admission gate: at most one running/submitted job, even across sessions.
    private static final AtomicBoolean BUSY = new AtomicBoolean();
    private MovementSync observedSync;
    private World observedWorld;
    private long generation, epoch, eventToken, revision;
    private HazardRecovery.Context observedContext;
    private long submittedRevision = -1;
    private long lastTick = Long.MIN_VALUE;
    private Report latest = new Report(Status.UNKNOWN, -1, -1, -1, 0, "Not observed");
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
            boolean changed = observedSync != sync || observedWorld != world
                    || generation != source.generation() || epoch != source.epoch()
                    || this.eventToken != eventToken || resourcesChanged(observedContext, context);
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
                WORKER.execute(() -> {
                    var budget = new HazardRecovery.Budget(HazardRecovery.DECISION_STEP_BUDGET);
                    try {
                        // No MovementSync lock is held across terrain queries or search.
                        var result = HazardRecovery.assess(world, source.state(), parameters, context, budget);
                        publish(sync, world, source, jobRevision, new Report(
                                result.admissible() ? Status.SAFE : Status.UNKNOWN,
                                source.tick(), source.generation(), source.epoch(), budget.steps(), result.reason()));
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
        if (revision != jobRevision || !current(sync, world, source)) {
            latest = new Report(Status.STALE, source.tick(), source.generation(), source.epoch(),
                    report.simulatedSteps(), "Discarded obsolete reserve observation");
            return;
        }
        latest = sync.physicsTickVersion() - source.tick() >= INTERVAL_TICKS
                ? new Report(Status.STALE, report.tick(), report.generation(), report.epoch(),
                        report.simulatedSteps(), "Reserve observation expired during search") : report;
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
