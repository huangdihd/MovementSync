package xin.bbtt.movements;

import org.joml.Vector3d;
import xin.bbtt.physics.*;
import xin.bbtt.world.World;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

/** Pure worker planning. Owners publish results only inside their navigation-generation guard. */
final class LowAltitudeFlightPlan {
    private static final ExecutorService WORKER = new ThreadPoolExecutor(1, 1, 0, TimeUnit.MILLISECONDS,
            new ArrayBlockingQueue<>(2), runnable -> {
        Thread thread = new Thread(runnable, "low-altitude-flight-planner");
        thread.setDaemon(true);
        return thread;
    });

    record Result(WaypointFlightTarget route, Vector3d origin, Vector3d destination, float yaw, float pitch) {}

    static CompletableFuture<Result> submit(World world, PhysicsState state, Vector3d destination,
                                             MovementParameters parameters, boolean gliding) {
        return submit(world, state, destination, parameters, gliding, false, false);
    }

    static CompletableFuture<Result> pursuit(World world, PhysicsState state, Vector3d destination,
                                              MovementParameters parameters, boolean gliding) {
        return submit(world, state, destination, parameters, gliding, false, true);
    }

    static CompletableFuture<Result> emergency(World world, PhysicsState state, MovementParameters parameters) {
        return submit(world, state, state.position(), parameters, true, true, false);
    }

    private static CompletableFuture<Result> submit(World world, PhysicsState state, Vector3d destination,
                                                    MovementParameters parameters, boolean gliding, boolean emergency, boolean pursuit) {
        Vector3d goal = new Vector3d(destination);
        if (!admissible(state.position(), goal)) return CompletableFuture.completedFuture(null);
        CompletableFuture<Result> result = new CompletableFuture<>();
        java.util.concurrent.FutureTask<Void> job = new java.util.concurrent.FutureTask<>(() -> {
            try {
                Vector3d delta = new Vector3d(goal).sub(state.position());
                float yaw = (float) Math.toDegrees(Math.atan2(-delta.x, delta.z));
                float pitch = 0;
                PhysicsState launch = gliding ? state : deployment(world, state, parameters, yaw, pitch);
                WaypointFlightTarget route = launch == null || Thread.currentThread().isInterrupted() ? null
                        : emergency ? emergencyRoute(world, launch, parameters)
                        : pursuit ? LowAltitudePlanner.planPursuit(world, launch, goal, parameters)
                        : LowAltitudePlanner.plan(world, launch, goal, parameters);
                result.complete(route == null ? null : new Result(route, state.position(), emergency ? route.goal() : goal, yaw, pitch));
            } catch (RuntimeException failure) {
                result.complete(null);
            }
            return null;
        });
        result.whenComplete((value, error) -> {
            if (result.isCancelled()) job.cancel(true);
        });
        try {
            WORKER.execute(job);
        } catch (java.util.concurrent.RejectedExecutionException busy) {
            result.complete(null);
        }
        return result;
    }

    private static WaypointFlightTarget emergencyRoute(World world, PhysicsState state, MovementParameters parameters) {
        // Keep the ordinary candidate ordering/support rules, filtering each site
        // by the same corridor planner rather than stopping at an unreachable first choice.
        WaypointFlightTarget[] accepted = {null};
        Vector3d selected = new LandingSearch().selectReachable(world, state, new int[0], false, parameters, site -> {
            if (Thread.currentThread().isInterrupted()) return false;
            var route = LowAltitudePlanner.plan(world, state, site, parameters);
            if (route == null || route.partial()
                    || route.waypoints().stream().anyMatch(point -> point.y > state.position().y)) return false;
            accepted[0] = route;
            return true;
        });
        return selected == null ? null : accepted[0];
    }

    /** Only reject invalid coordinates; distant destinations use rolling partial routes. */
    static boolean admissible(Vector3d origin, Vector3d goal) {
        return origin.isFinite() && goal.isFinite();
    }

    /** Preview the actual stationary jump and descending deployment, never a fabricated upward impulse. */
    static PhysicsState deployment(World world, PhysicsState initial, MovementParameters parameters,
                                   float yaw, float pitch) {
        PhysicsState state = initial;
        for (int tick = 0; tick < 40; tick++) {
            if (Thread.currentThread().isInterrupted()) return null;
            if (!state.onGround() && state.velocity().y <= 0) {
                PhysicsState deployed = state;
                for (int glide = 0; glide < 5; glide++) {
                    PhysicsResult next = new GlidingPhysics(yaw, pitch, false, PlayerInput.NONE, parameters)
                            .step(world, deployed, (double) 0.6f / 2, (double) 0.6f);
                    if (!safe(world, next) || next.onGround()) return null;
                    deployed = state(next);
                }
                return state;
            }
            PhysicsResult next = new WalkingPhysics(PlayerInput.NONE.withJump(tick == 0 && initial.onGround()),
                    yaw, true, parameters).step(world, state, (double) 0.6f / 2, (double) 1.8f);
            if (!safe(world, next) || next.onGround()) return null;
            state = state(next);
        }
        return null;
    }

    private static PhysicsState state(PhysicsResult result) {
        return new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance(), result.pose());
    }

    private static boolean safe(World world, PhysicsResult result) {
        Vector3d p = result.position();
        var environment = EnvironmentalPhysics.sample(world, p, (double) 0.6f / 2, (double) 1.8f);
        return FlightHazards.safeLanding(world, p) && !result.collidedX() && !result.collidedZ()
                && world.chunkLoaded((int) Math.floor(p.x) >> 4, (int) Math.floor(p.z) >> 4)
                && environment.waterHeight() == 0 && environment.lavaHeight() == 0;
    }
}
