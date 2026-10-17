package xin.bbtt.physics;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Bounded worker-side geometric 3D A*. Nodes are positions, never flight states.
 * A route is a conservative terrain corridor, not a guarantee of dynamic reachability:
 * the controller must validate its actual trajectory, especially near a landing.
 * Unknown terrain is blocked. Budget exhaustion returns the closest reached loaded
 * position to the goal, rather than rejecting distant destinations. */
public final class LowAltitudePlanner {
    static final double HALF_WIDTH = PlayerPose.HALF_WIDTH;
    static final double HEIGHT = PlayerPose.GLIDING.height();
    // Worker engineering budgets, not physical reachability thresholds: retain
    // at most 4096 expanded cells and 160000 collision/LOS probes per request.
    // Two-block spacing covers several hundred loaded blocks cheaply; narrower
    // off-grid passages can be missed, so failure never licenses unknown space.
    private static final int MAX_EXPANSIONS = 4096;
    private static final int MAX_CHECKS = 160000;
    private static final double GRID = 2;
    private static final World EMPTY = new World();
    private record Clearance(double horizontal, double vertical, double corridor) {}
    private static final ThreadPoolExecutor WORKER = new ThreadPoolExecutor(1, 1, 0,
            TimeUnit.MILLISECONDS, new ArrayBlockingQueue<>(2), runnable -> {
                Thread thread = new Thread(runnable, "low-altitude-planner");
                thread.setDaemon(true);
                return thread;
            }, new ThreadPoolExecutor.AbortPolicy());
    private record Key(int x, int y, int z) {}
    private record Node(Key key, Vector3d position, Node parent, double cost, double estimate) {}
    private static final class Budget {
        int checks;
        final CompletableFuture<?> cancellation;
        Budget(CompletableFuture<?> cancellation) { this.cancellation = cancellation; }
        boolean cancelled() {
            return Thread.currentThread().isInterrupted() || cancellation != null && cancellation.isCancelled();
        }
        boolean take() { return !cancelled() && ++checks <= MAX_CHECKS; }
    }

    public LowAltitudePlanner() {}

    /** Clearance for one control interval, derived from a ninety-degree turn and
     * pull-up, rather than applying a full turn radius symmetrically at every cell.
     * Six boosted travel ticks bound the local corridor neighbourhood considered by
     * the six-tick controller; every admitted expansion is still terrain-validated.
     */
    private static Clearance clearance(PhysicsState initial, MovementParameters movement) {
        PhysicsState probe = new PhysicsState(new Vector3d(0, 128, 0), initial.velocity(), false,
                initial.fallDistance(), PlayerPose.GLIDING);
        float heading = yaw(initial.velocity());
        PhysicsResult turn = new GlidingPhysics(heading + 90, -30, false, PlayerInput.NONE, movement)
                .step(EMPTY, probe, HALF_WIDTH, HEIGHT);
        double horizontal = Math.abs(turn.position().sub(probe.position()).dot(
                new Vector3d(Math.cos(Math.toRadians(heading + 90)), 0, Math.sin(Math.toRadians(heading + 90)))));
        double vertical = Math.max(0, probe.position().y - turn.position().y);
        PhysicsState boosted = probe;
        double reach = HEIGHT;
        for (int tick = 0; tick < 6; tick++) {
            boosted = FlightSafety.nextState(new GlidingPhysics(heading, 0, true, PlayerInput.NONE, movement)
                    .step(EMPTY, boosted, HALF_WIDTH, HEIGHT));
            reach = Math.max(reach, boosted.position().distance(probe.position()));
        }
        return new Clearance(horizontal, vertical, reach + HALF_WIDTH);
    }

    public static WaypointFlightTarget plan(World world, PhysicsState initial, Vector3d goal,
            MovementParameters movement) {
        return search(world, initial, new Vector3d(Objects.requireNonNull(goal)), movement, new Budget(null));
    }

    /** A moving ground endpoint is an airborne pursuit intent, never a landing site.
     * Lift by the body clearance plus the unpowered drop over one steering horizon;
     * terrain validation, rather than a world cruise altitude, decides feasibility. */
    public static WaypointFlightTarget planPursuit(World world, PhysicsState initial, Vector3d goal,
            MovementParameters movement) {
        Clearance margin = clearance(initial, movement);
        PhysicsState probe = new PhysicsState(new Vector3d(0, 128, 0), initial.velocity(), false,
                0, PlayerPose.GLIDING);
        double drop = 0;
        float heading = yaw(initial.velocity());
        for (int tick = 0; tick < 6; tick++) {
            probe = FlightSafety.nextState(new GlidingPhysics(heading, 0, false, PlayerInput.NONE, movement)
                    .step(EMPTY, probe, HALF_WIDTH, HEIGHT));
            drop = Math.max(drop, 128 - probe.position().y);
        }
        Vector3d airborne = new Vector3d(goal).add(0, HEIGHT + margin.vertical() + drop, 0);
        // A packet-interpolated entity or its follow offset can be inside a
        // ridge. Find loaded air above that column, never aim into the solid.
        if (!airborne.isFinite()) return null;
        while (!clear(world, airborne, airborne, margin.horizontal(), margin.vertical())) {
            airborne.y += 1;
            if (airborne.y + HEIGHT + margin.vertical() >= world.getMaxWorldY()
                    || !world.chunkLoaded((int) Math.floor(airborne.x) >> 4,
                            (int) Math.floor(airborne.z) >> 4)) return null;
        }
        WaypointFlightTarget route = search(world, initial, airborne, movement, new Budget(null));
        return route == null ? null : route.asPursuit();
    }

    /** One daemon worker, two queued requests; a saturated queue returns null. */
    public static CompletableFuture<WaypointFlightTarget> planAsync(World world, PhysicsState initial,
            Vector3d goal, MovementParameters movement) {
        Vector3d destination = new Vector3d(Objects.requireNonNull(goal));
        CompletableFuture<WaypointFlightTarget> future = new CompletableFuture<>();
        try {
            WORKER.execute(() -> {
                if (future.isCancelled()) return;
                try {
                    future.complete(search(world, initial, destination, movement, new Budget(future)));
                } catch (RuntimeException failure) {
                    future.completeExceptionally(failure);
                }
            });
        } catch (java.util.concurrent.RejectedExecutionException busy) {
            future.complete(null);
        }
        return future;
    }

    private static WaypointFlightTarget search(World world, PhysicsState initial, Vector3d goal,
            MovementParameters movement, Budget budget) {
        Objects.requireNonNull(world);
        Objects.requireNonNull(initial);
        Objects.requireNonNull(movement);
        if (!goal.isFinite() || !initial.position().isFinite() || !initial.velocity().isFinite()
                || !Double.isFinite(initial.fallDistance()) || initial.onGround()) return null;
        Vector3d origin = initial.position();
        Clearance clearance = clearance(initial, movement);
        double horizontal = clearance.horizontal(), vertical = clearance.vertical();
        if (!clear(world, origin, origin, horizontal, vertical)) return null;
        boolean landing = landing(world, goal);
        boolean blockedLanding = supported(world, goal) && !landing;
        PriorityQueue<Node> open = new PriorityQueue<>(Comparator.comparingDouble(Node::estimate));
        Map<Key, Double> costs = new HashMap<>();
        Map<Key, Boolean> cells = new HashMap<>();
        Node start = new Node(new Key(0, 0, 0), origin, null, 0, origin.distance(goal));
        Node best = start;
        open.add(start);
        costs.put(start.key(), 0.0);
        for (int expanded = 0; !open.isEmpty() && expanded < MAX_EXPANSIONS
                && budget.checks < MAX_CHECKS; expanded++) {
            if (budget.cancelled()) return null;
            Node node = open.remove();
            if (node.cost() > costs.getOrDefault(node.key(), Double.POSITIVE_INFINITY)) continue;
            if (node.position().distanceSquared(goal) < best.position().distanceSquared(goal)) best = node;
            // Short terminal connections only: no unbounded distant LOS query.
            if (!blockedLanding && node.position().distance(goal) <= GRID * 2
                    && lineOfSight(world, node.position(), goal, landing ? 0 : horizontal,
                            landing ? 0 : vertical, budget)
                    && (!landing || landing(world, goal))) {
                Node end = new Node(node.key(), new Vector3d(goal), node,
                        node.cost() + node.position().distance(goal), 0);
                return route(world, end, goal, landing, false, horizontal, vertical, clearance.corridor(), budget);
            }
            for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    if (!budget.take()) break;
                    Key key = new Key(node.key().x() + dx, node.key().y() + dy, node.key().z() + dz);
                    Vector3d p = new Vector3d(origin).add(key.x() * GRID, key.y() * GRID, key.z() * GRID);
                    double cost = node.cost() + GRID * Math.sqrt(dx * dx + dy * dy + dz * dz);
                    if (cost >= costs.getOrDefault(key, Double.POSITIVE_INFINITY)) continue;
                    if (!cells.computeIfAbsent(key, k -> clear(world, p, p, horizontal, vertical))) continue;
                    if (!lineOfSight(world, node.position(), p, horizontal, vertical, budget)) continue;
                    Node child = new Node(key, p, node, cost, cost + p.distance(goal));
                    costs.put(key, cost);
                    open.add(child);
                    // Include discovered frontier nodes when the budget runs out.
                    if (p.distanceSquared(goal) < best.position().distanceSquared(goal)) best = child;
                }
        }
        if (budget.cancelled() || best == start) return null;
        return route(world, best, goal, false, true, horizontal, vertical, clearance.corridor(), budget);
    }

    static boolean lineOfSight(World world, Vector3d from, Vector3d to, double horizontal, double vertical) {
        return lineOfSight(world, from, to, horizontal, vertical, new Budget(null));
    }

    private static boolean lineOfSight(World world, Vector3d from, Vector3d to,
            double horizontal, double vertical, Budget budget) {
        if (!from.isFinite() || !to.isFinite()) return false;
        double length = from.distance(to);
        if (!Double.isFinite(length) || length > MAX_CHECKS) return false;
        int steps = Math.max(1, (int) Math.ceil(length));
        Vector3d previous = new Vector3d(from);
        for (int i = 1; i <= steps; i++) {
            if (!budget.take()) return false;
            Vector3d next = new Vector3d(from).lerp(to, (double) i / steps);
            if (!clear(world, previous, next, horizontal, vertical)) return false;
            previous = next;
        }
        return true;
    }

    /** Swept boxes deliberately overapproximate each short diagonal, so thin collision
     * shapes cannot fall between samples. Exact float body, not the old .299 proxy. */
    static boolean clear(World world, Vector3d from, Vector3d to, double horizontal, double vertical) {
        return world.readTerrain(() -> clearLocked(world, from, to, horizontal, vertical));
    }

    private static boolean clearLocked(World world, Vector3d from, Vector3d to, double horizontal, double vertical) {
        double radius = HALF_WIDTH + horizontal;
        double minX = Math.min(from.x, to.x) - radius, maxX = Math.max(from.x, to.x) + radius;
        double minY = Math.min(from.y, to.y) - vertical, maxY = Math.max(from.y, to.y) + HEIGHT + vertical;
        double minZ = Math.min(from.z, to.z) - radius, maxZ = Math.max(from.z, to.z) + radius;
        if (minY < World.getMinWorldY() || maxY > World.getMaxWorldY()) return false;
        // Also bounds pathological input velocities before any large world scan.
        if (maxX - minX > 128 || maxY - minY > 128 || maxZ - minZ > 128) return false;
        for (int x = (int) Math.floor(minX) >> 4; x <= (int) Math.floor(maxX) >> 4; x++)
            for (int z = (int) Math.floor(minZ) >> 4; z <= (int) Math.floor(maxZ) >> 4; z++)
                if (!world.chunkLoaded(x, z)) return false;
        if (world.isBoxColliding(minX, minY, minZ, maxX, maxY, maxZ)) return false;
        // Reuse the shared hazard policy. Probe each touched cell with a gliding
        // body at its centre; this covers the exact body/margin slivers as well.
        for (int x = (int) Math.floor(minX); x < Math.ceil(maxX); x++)
            for (int y = (int) Math.floor(minY); y < Math.ceil(maxY); y++)
                for (int z = (int) Math.floor(minZ); z < Math.ceil(maxZ); z++) {
                    PhysicsState cell = new PhysicsState(new Vector3d(x + .5, y + .01, z + .5),
                            new Vector3d(), false, 0, PlayerPose.GLIDING);
                    PhysicsResult result = new PhysicsResult(cell.position(), cell.velocity(), false, false, false, 0,
                            PlayerPose.GLIDING);
                    if (FlightHazards.damage(world, cell, result, DamageParameters.NONE, true).raw() > 0
                            || FlightHazards.damage(world, cell, result, DamageParameters.NONE, false).raw() > 0) return false;
                }
        return true;
    }

    private static boolean supported(World world, Vector3d goal) {
        if (!world.chunkLoaded((int) Math.floor(goal.x) >> 4, (int) Math.floor(goal.z) >> 4)) return false;
        return world.isBoxColliding(goal.x - HALF_WIDTH, goal.y - .01, goal.z - HALF_WIDTH,
                goal.x + HALF_WIDTH, goal.y, goal.z + HALF_WIDTH);
    }

    private static boolean landing(World world, Vector3d goal) {
        return clear(world, goal, goal, 0, 0) && supported(world, goal)
                && !world.isBoxColliding(goal.x - HALF_WIDTH, goal.y, goal.z - HALF_WIDTH,
                        goal.x + HALF_WIDTH, goal.y + (double) 1.8f, goal.z + HALF_WIDTH)
                && FlightHazards.safeLanding(world, goal);
    }

    static PhysicsState preview(World world, PhysicsState state, float yaw, float pitch,
            MovementParameters movement, FlightSafety safety) {
        return preview(world, state, yaw, pitch, movement, safety, false);
    }

    static PhysicsState preview(World world, PhysicsState state, float yaw, float pitch,
            MovementParameters movement, FlightSafety safety, boolean attached) {
        state = new PhysicsState(state.position(), state.velocity(), state.onGround(), state.fallDistance(), PlayerPose.GLIDING);
        PhysicsResult result = new GlidingPhysics(yaw, pitch, attached, PlayerInput.NONE, movement)
                .step(world, state, HALF_WIDTH, HEIGHT);
        if (!FlightSafety.knownStep(world, state, result) || result.collidedX() || result.collidedZ()
                || !lineOfSight(world, state.position(), result.position(), 0, 0)) return null;
        if (result.onGround() && (!landing(world, result.position())
                || safety.collisionDamage(world, state, result, yaw, pitch, attached).raw() > 0)) return null;
        return new PhysicsState(result.position(), result.velocity(), result.onGround(), result.fallDistance(),
                result.onGround() ? PlayerPose.STANDING : PlayerPose.GLIDING);
    }

    static float yaw(Vector3d delta) { return (float) Math.toDegrees(Math.atan2(-delta.x, delta.z)); }
    static boolean nearGoal(Vector3d position, Vector3d goal) {
        return Math.hypot(position.x - goal.x, position.z - goal.z) <= 1 && Math.abs(position.y - goal.y) <= .5;
    }

    private static WaypointFlightTarget route(World world, Node end, Vector3d goal, boolean landing,
            boolean partial, double horizontal, double vertical, double corridor, Budget budget) {
        List<Vector3d> points = new ArrayList<>();
        for (Node node = end; node != null; node = node.parent()) points.add(node.position());
        Collections.reverse(points);
        List<Vector3d> smooth = new ArrayList<>();
        smooth.add(points.get(0));
        // Greedy LOS smoothing uses the same budget; exhausted searches keep their
        // already validated edges rather than spending unbounded work smoothing.
        for (int i = 0; i < points.size() - 1;) {
            int next = i + 1;
            for (int candidate = i + 2; candidate < points.size(); candidate++) {
                if (landing && candidate == points.size() - 1) break;
                if (!lineOfSight(world, points.get(i), points.get(candidate), horizontal, vertical, budget)) break;
                next = candidate;
            }
            smooth.add(points.get(next));
            i = next;
        }
        // Expansion is checked against real swept body geometry on demand. Near
        // support the downward allowance naturally tapers to zero, not the lateral
        // allowance: landing is a safe volume, never an exact centreline.
        return new WaypointFlightTarget(smooth, goal, landing, partial, corridor, world);
    }
}
