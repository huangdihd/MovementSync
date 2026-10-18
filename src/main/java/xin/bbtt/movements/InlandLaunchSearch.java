package xin.bbtt.movements;

import org.joml.Vector3d;
import org.joml.Vector3i;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.pathfinding.StandablePositionResolver;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.*;
import java.util.function.Function;

/** One bounded, cancellable search; neither pathfinding nor physics runs on the movement tick. */
final class InlandLaunchSearch {
    private static final ExecutorService WORKER = new ThreadPoolExecutor(1, 1, 0, TimeUnit.MILLISECONDS,
            new ArrayBlockingQueue<>(1), runnable -> {
                Thread thread = new Thread(runnable, "inland-launch-search");
                thread.setDaemon(true);
                return thread;
            });
    record Result(Vector3i destination, Vector3d origin, List<PathStep> path, float yaw) {}
    private record Candidate(Vector3i node, Vector3d feet, double score) {}

    static CompletableFuture<Result> submit(World world, Vector3d origin, Vector3d goal,
            MovementParameters parameters, HazardRecovery.Context resources,
            Function<Vector3i, List<PathStep>> pathfinder) {
        Vector3d start = new Vector3d(origin);
        CompletableFuture<Result> result = new CompletableFuture<>();
        FutureTask<Void> job = new FutureTask<>(() -> {
            try {
                result.complete(search(world, start, goal, parameters, resources, pathfinder));
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
                result.cancel(false);
            } catch (Exception failure) {
                result.complete(null);
            }
            return null;
        });
        result.whenComplete((value, error) -> { if (result.isCancelled()) job.cancel(true); });
        try { WORKER.execute(job); }
        catch (RejectedExecutionException busy) { result.complete(null); }
        return result;
    }

    private static Result search(World world, Vector3d start, Vector3d goal,
            MovementParameters parameters, HazardRecovery.Context resources,
            Function<Vector3i, List<PathStep>> pathfinder) throws Exception {
        long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(120);
        // Sum nearby unsafe/unsupported directions. Ordering is only a heuristic, never a gate.
        Vector3d away = new Vector3d();
        for (int dx = -6; dx <= 6; dx += 2) for (int dz = -6; dz <= 6; dz += 2) {
            if (dx == 0 && dz == 0) continue;
            int x = (int) Math.floor(start.x) + dx, z = (int) Math.floor(start.z) + dz;
            if (StandablePositionResolver.feetY(world, x, StandablePositionResolver.nodeY(start.y), z).isEmpty()
                    || !FlightHazards.safeLanding(world, new Vector3d(x + .5, start.y, z + .5)))
                away.add(-dx / (double) (dx * dx + dz * dz), 0, -dz / (double) (dx * dx + dz * dz));
        }
        if (away.lengthSquared() > 0) away.normalize();
        List<Candidate> candidates = new ArrayList<>();
        int ox = (int) Math.floor(start.x), oz = (int) Math.floor(start.z);
        int oy = StandablePositionResolver.nodeY(start.y);
        for (int dx = -16; dx <= 16; dx += 2) for (int dz = -16; dz <= 16; dz += 2) {
            double distance = Math.hypot(dx, dz);
            if (distance < 8 || distance > 16) continue;
            if (Thread.currentThread().isInterrupted()) throw new InterruptedException();
            int x = ox + dx, z = oz + dz;
            if (!world.chunkLoaded(x >> 4, z >> 4)) continue;
            for (int y = oy - 1; y <= oy + 1; y++) {
                var feetY = StandablePositionResolver.feetY(world, x, y, z);
                if (feetY.isEmpty()) continue;
                Vector3d feet = new Vector3d(x + .5, feetY.getAsDouble(), z + .5);
                if (!FlightHazards.safeLanding(world, feet)) continue;
                candidates.add(new Candidate(new Vector3i(x, y, z), feet,
                        -(dx * away.x + dz * away.z) + distance * .1));
            }
        }
        candidates.sort(Comparator.comparingDouble(Candidate::score));
        int attempts = 0;
        for (Candidate candidate : candidates) {
            if (Thread.currentThread().isInterrupted()) throw new InterruptedException();
            if (++attempts > TravelMovement.MAX_TAKEOFF_PATH_ATTEMPTS || System.nanoTime() >= deadline) break;
            List<PathStep> path = pathfinder.apply(candidate.node());
            Vector3i node = candidate.node();
            if (path == null || path.size() <= 1 || !path.get(path.size() - 1).getNode()
                    .equals(new Node(node.x, node.y, node.z))) continue;
            Vector3d delta = new Vector3d(goal).sub(candidate.feet());
            float direct = (float) Math.toDegrees(Math.atan2(-delta.x, delta.z));
            for (float offset : new float[]{0, 90, -90, 180}) {
                long remaining = deadline - System.nanoTime();
                if (remaining <= 0) return null;
                var proof = LowAltitudeFlightPlan.submitHeading(world,
                        new PhysicsState(candidate.feet(), new Vector3d(), true), goal,
                        parameters, resources, direct + offset);
                try {
                    var plan = proof.get(remaining, TimeUnit.NANOSECONDS);
                    if (plan != null && proof.status() == LowAltitudeFlightPlan.Status.COMPLETE)
                        return new Result(new Vector3i(node), start, List.copyOf(path), plan.yaw());
                } finally {
                    if (!proof.isDone()) proof.cancel(true);
                }
            }
        }
        return null;
    }
}
