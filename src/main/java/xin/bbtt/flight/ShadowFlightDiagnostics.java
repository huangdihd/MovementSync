package xin.bbtt.flight;

import java.util.Arrays;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

/** Bounded observer: no commands or admission calls. */
public final class ShadowFlightDiagnostics {
    public record Comparison(long tick, long generation, long epoch,
            float actualYaw, float actualPitch, Float shadowYaw, Float shadowPitch,
            int simulatedSteps, long nanos, long p50Nanos, long p95Nanos, String status) {}
    private static final java.util.concurrent.ExecutorService WORKER = Executors.newSingleThreadExecutor(r -> {
        Thread thread = new Thread(r, "flight-shadow");
        thread.setDaemon(true);
        return thread;
    });
    private static final AtomicBoolean BUSY = new AtomicBoolean();
    private final long[] durations = new long[128];
    private int samples;
    private long lastTick = Long.MIN_VALUE;
    private volatile Comparison latest;
    private LocalAirRoute workerRoute;
    private java.util.List<org.joml.Vector3d> workerPoints;
    private long workerGeneration = -1;
    private volatile java.util.concurrent.CompletableFuture<Void> completion =
            java.util.concurrent.CompletableFuture.completedFuture(null);

    /** Wait for the currently submitted observation, including discarded results. */
    boolean awaitIdle(long timeout, java.util.concurrent.TimeUnit unit)
            throws InterruptedException, java.util.concurrent.ExecutionException,
            java.util.concurrent.TimeoutException {
        completion.get(timeout, unit);
        return true;
    }
    public Comparison latest() { return latest; }

    /** Planner thread only. At most one observation per ten physics ticks; no backlog. */
    public synchronized void submit(MovementSync sync, FlightControlAdmission.Source source,
            WaypointFlightTarget route, ElytraFlightController.Decision actual) {
        if (!sync.getFlightScopeRouter().isShadowEnabled() || Thread.holdsLock(sync)
                || route == null || source.tick() == lastTick
                || lastTick != Long.MIN_VALUE && source.tick() - lastTick < 10
                || !BUSY.compareAndSet(false, true)) return;
        lastTick = source.tick();
        World world = sync.getWorld();
        var parameters = sync.getMovementParameters();
        boolean attached = sync.hasAttachedFirework();
        var routeSnapshot = new LocalAirRoute(route);
        var finished = new java.util.concurrent.CompletableFuture<Void>();
        completion = finished;
        WORKER.execute(() -> {
            try {
                var routePoints = routeSnapshot.waypoints();
                if (workerRoute == null || workerGeneration != source.generation()
                        || !routePoints.equals(workerPoints)) {
                    workerRoute = routeSnapshot;
                    workerPoints = routePoints;
                    workerGeneration = source.generation();
                }
                var points = workerRoute.lookahead(world, source.state());
                var result = points.isEmpty()
                        ? new LocalLookaheadSolver.Result(null, 0, 0,
                                LocalLookaheadSolver.Status.NO_SAFE_CANDIDATE, null, Double.NEGATIVE_INFINITY)
                        : new LocalLookaheadSolver().solve(points, source.state(), world, parameters,
                                actual.yaw(), actual.pitch(), attached,
                                new LocalLookaheadSolver.Budget(600, 20, 25_000_000));
                if (!sync.getFlightScopeRouter().isShadowEnabled()
                        || sync.getWorld() != world || sync.getNavigationGeneration() != source.generation()
                        || sync.flightControlEpoch() != source.epoch()) return;
                var selected = result.decision();
                long[] sorted;
                synchronized (this) {
                    durations[samples++ % durations.length] = result.elapsedNanos();
                    sorted = Arrays.copyOf(durations, Math.min(samples, durations.length));
                }
                Arrays.sort(sorted);
                latest = new Comparison(source.tick(), source.generation(), source.epoch(),
                        actual.yaw(), actual.pitch(), selected == null ? null : selected.yaw(),
                        selected == null ? null : selected.pitch(), result.steps(), result.elapsedNanos(),
                        sorted[(sorted.length - 1) / 2], sorted[(int) Math.ceil(sorted.length * .95) - 1],
                        result.status().toString());
                MovementSync.getLogger().info("Flight shadow {}", latest);
            } catch (RuntimeException error) {
                MovementSync.getLogger().debug("Flight shadow observation failed", error);
            } finally {
                BUSY.set(false);
                finished.complete(null);
            }
        });
    }
}
