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

    /** Worker outcome, not a claim about the reason a particular terrain cell was blocked. */
    enum Status { PENDING, COMPLETE, PARTIAL, INVALID_COORDINATES, DEPLOYMENT_REJECTED,
        NO_CORRIDOR, CANCELLED, BUSY, FAILED }

    static final class PlanFuture extends CompletableFuture<Result> {
        private volatile Status status = Status.PENDING;
        private volatile String detail;
        String detail() { return detail; }
        Status status() { return isCancelled() ? Status.CANCELLED : status; }
        private void finish(Result result, Status outcome) {
            status = outcome;
            complete(result);
        }
    }

    static PlanFuture submit(World world, PhysicsState state, Vector3d destination,
                                             MovementParameters parameters, boolean gliding) {
        return submit(world, state, destination, parameters, gliding, HazardRecovery.Context.EMPTY);
    }

    static PlanFuture submit(World world, PhysicsState state, Vector3d destination,
            MovementParameters parameters, boolean gliding, HazardRecovery.Context context) {
        return submit(world, state, destination, parameters, gliding, false, false, context);
    }

    static CompletableFuture<Result> pursuit(World world, PhysicsState state, Vector3d destination,
                                              MovementParameters parameters, boolean gliding) {
        return pursuit(world, state, destination, parameters, gliding, HazardRecovery.Context.EMPTY);
    }

    static CompletableFuture<Result> pursuit(World world, PhysicsState state, Vector3d destination,
            MovementParameters parameters, boolean gliding, HazardRecovery.Context context) {
        return submit(world, state, destination, parameters, gliding, false, true, context);
    }

    static CompletableFuture<Result> emergency(World world, PhysicsState state, MovementParameters parameters) {
        return emergency(world, state, parameters, HazardRecovery.Context.EMPTY);
    }

    static CompletableFuture<Result> emergency(World world, PhysicsState state, MovementParameters parameters,
            HazardRecovery.Context context) {
        return submit(world, state, state.position(), parameters, true, true, false, context);
    }

    private static PlanFuture submit(World world, PhysicsState state, Vector3d destination,
                                                    MovementParameters parameters, boolean gliding, boolean emergency, boolean pursuit, HazardRecovery.Context context) {
        return submit(world, state, destination, parameters, gliding, emergency, pursuit, context, null);
    }

    /** Same full launch/route proof, with an explicitly selected relocation launch heading. */
    static PlanFuture submitHeading(World world, PhysicsState state, Vector3d destination,
            MovementParameters parameters, HazardRecovery.Context context, float yaw) {
        return submit(world, state, destination, parameters, false, false, false, context, yaw);
    }

    private static PlanFuture submit(World world, PhysicsState state, Vector3d destination,
            MovementParameters parameters, boolean gliding, boolean emergency, boolean pursuit,
            HazardRecovery.Context context, Float launchYaw) {
        Vector3d goal = new Vector3d(destination);
        PlanFuture result = new PlanFuture();
        if (!admissible(state.position(), goal)) {
            result.finish(null, Status.INVALID_COORDINATES);
            return result;
        }
        java.util.concurrent.FutureTask<Void> job = new java.util.concurrent.FutureTask<>(() -> {
            try {
                Vector3d delta = new Vector3d(goal).sub(state.position());
                float yaw = launchYaw == null ? (float) Math.toDegrees(Math.atan2(-delta.x, delta.z)) : launchYaw;
                float pitch = 0;
                var abort = gliding ? null : launchYaw == null
                        ? DeploymentSafety.assess(world, state, parameters, yaw, pitch)
                        : DeploymentSafety.assessRelocation(world, state, parameters, yaw, pitch);
                PhysicsState launch = gliding ? state : !abort.safe() ? null
                        : deployment(world, state, parameters, yaw, pitch);
                if (abort != null && !abort.safe()) result.detail = abort.reason();
                WaypointFlightTarget route = launch == null || Thread.currentThread().isInterrupted() ? null
                        : emergency ? emergencyRoute(world, launch, parameters, context)
                        : pursuit ? LowAltitudePlanner.planPursuit(world, launch, goal, parameters, context)
                        : LowAltitudePlanner.plan(world, launch, goal, parameters, context);
                if (launch != null && route == null) {
                    var reserve = HazardRecovery.assess(world, launch, parameters, context);
                    result.detail = reserve.admissible() ? "no validated corridor within search budget"
                            : reserve.reason();
                }
                Result planned = route == null ? null
                        : new Result(route, state.position(), emergency ? route.goal() : goal, yaw, pitch);
                result.finish(planned, Thread.currentThread().isInterrupted() ? Status.CANCELLED
                        : launch == null ? Status.DEPLOYMENT_REJECTED
                        : route == null ? Status.NO_CORRIDOR
                        : route.partial() ? Status.PARTIAL : Status.COMPLETE);
            } catch (RuntimeException failure) {
                result.finish(null, Status.FAILED);
            }
            return null;
        });
        result.whenComplete((value, error) -> {
            if (result.isCancelled()) job.cancel(true);
        });
        try {
            WORKER.execute(job);
        } catch (java.util.concurrent.RejectedExecutionException busy) {
            result.finish(null, Status.BUSY);
        }
        return result;
    }

    private static WaypointFlightTarget emergencyRoute(World world, PhysicsState state,
            MovementParameters parameters, HazardRecovery.Context context) {
        // Every candidate, including remembered ground, needs a complete loaded corridor.
        WaypointFlightTarget[] accepted = {null};
        java.util.function.Predicate<Vector3d> reachable = site -> {
            if (Thread.currentThread().isInterrupted()) return false;
            var route = LowAltitudePlanner.plan(world, state, site, parameters, context);
            if (route == null || route.partial()) return false;
            accepted[0] = route;
            return true;
        };
        Vector3d remembered = context.memory() == null ? null : context.memory().nearest(state.position());
        if (remembered != null) {
            Vector3d site = ElytraFlightController.landingSite(world, remembered.x, remembered.z, remembered.y);
            if (site != null && reachable.test(site)) return accepted[0];
        }
        Vector3d selected = new LandingSearch().selectReachable(world, state,
                context.rocketFlightDurations(), context.attached(), parameters, reachable);
        return selected == null ? null : accepted[0];
    }

    /** Only reject invalid coordinates; distant destinations use rolling partial routes. */
    static boolean admissible(Vector3d origin, Vector3d goal) {
        return origin.isFinite() && goal.isFinite();
    }

    /**
     * Success-only preview of the actual stationary jump and descending deployment.
     * Not launch authorization: submit and lifecycle init must also pass DeploymentSafety,
     * including no-deploy, interruption, and independent delayed-metadata landing branches.
     */
    static PhysicsState deployment(World world, PhysicsState initial, MovementParameters parameters,
                                   float yaw, float pitch) {
        PhysicsState state = initial;
        for (int tick = 0; tick < 40; tick++) {
            if (Thread.currentThread().isInterrupted()) return null;
            if (!state.onGround() && state.velocity().y <= 0) {
                PhysicsState deployed = state;
                for (int glide = 0; glide < 5; glide++) {
                    PhysicsResult next = new GlidingPhysics(yaw, pitch, false, PlayerInput.NONE, parameters)
                            .step(world, deployed, (double) 0.6f / 2, glide == 0 ? (double) 1.8f : (double) 0.6f);
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
