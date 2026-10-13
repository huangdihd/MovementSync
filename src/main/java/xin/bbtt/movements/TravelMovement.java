package xin.bbtt.movements;

import org.joml.Vector3d;
import org.joml.Vector3i;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.LangManager;
import xin.bbtt.movement.Movement;
import xin.bbtt.pathfinding.DStarLite;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.pathfinding.StandablePositionResolver;
import xin.bbtt.world.World;

import java.util.Comparator;
import java.util.ArrayList;
import java.util.List;

/** A single navigation generation owns all legs, including landing before ground fallback. */
public final class TravelMovement extends Movement {
    public static final double FLIGHT_DISTANCE_THRESHOLD = 96;
    static final int TAKEOFF_SEARCH_RADIUS = 16;
    static final int MAX_TAKEOFF_PATH_ATTEMPTS = 16;

    interface Legs {
        List<PathStep> plan(Vector3i destination);
        Movement walk(List<PathStep> path, Vector3i destination, boolean intermediate);
        Movement flight();
    }

    private final Legs legs;
    private enum Stage { PREPARE, APPROACH, FLIGHT, LAND, WALK }
    private final Vector3i goal;
    private final long generation;
    private final boolean fly;
    private Stage stage = Stage.PREPARE;
    private volatile Movement child;
    private volatile boolean stopped;

    public TravelMovement(Vector3i goal, long generation, boolean fly) {
        this(goal, generation, fly, null);
    }

    TravelMovement(Vector3i goal, long generation, boolean fly, Legs legs) {
        this.goal = new Vector3i(goal);
        this.generation = generation;
        this.fly = fly;
        this.legs = legs == null ? new Legs() {
            @Override public List<PathStep> plan(Vector3i destination) { return pathTo(destination); }
            @Override public Movement walk(List<PathStep> path, Vector3i destination, boolean intermediate) {
                PathMovement movement = new PathMovement(path, -1, false, destination, -1, generation);
                return intermediate ? movement.asIntermediateLeg(TravelMovement.this.goal) : movement;
            }
            @Override public Movement flight() {
                Vector3i destination = TravelMovement.this.goal;
                return new ElytraFlyMovement(new Vector3d(destination.x + 0.5, destination.y, destination.z + 0.5), generation);
            }
        } : legs;
    }

    public static boolean shouldFly(double distance, boolean elytra, boolean fireworks,
                                    boolean walk, boolean gliding) {
        return !walk && elytra && (gliding || distance > FLIGHT_DISTANCE_THRESHOLD && fireworks);
    }

    /** A 3x3 launch column reserves clear sky and lateral room for the climb. */
    public static boolean isOpen(World world, Vector3d feet) {
        for (int x = (int) Math.floor(feet.x - 1.5); x <= (int) Math.floor(feet.x + 1.5); x++) {
            for (int z = (int) Math.floor(feet.z - 1.5); z <= (int) Math.floor(feet.z + 1.5); z++) {
                if (!world.chunkLoaded(x >> 4, z >> 4)) return false;
            }
        }
        return !world.isBoxColliding(feet.x - 1.5, feet.y + 0.01, feet.z - 1.5,
                feet.x + 1.5, World.getMaxWorldY() + 1.0, feet.z + 1.5);
    }

    @Override
    public void init() {}

    @Override
    public void onTick() {
        MovementSync movementSync = MovementSync.INSTANCE;
        if (stopped || finished || !movementSync.isNavigationRequestCurrent(generation)) return;
        Movement currentChild = child;
        if (currentChild != null) {
            currentChild.onTick();
            if (!currentChild.isFinished()) return;
            child = null;
            currentChild.onStop();
            if (stopped || finished || !movementSync.isNavigationRequestCurrent(generation)) return;
            if (stage == Stage.WALK) {
                finished = true;
                return;
            }
            if (stage == Stage.APPROACH && isOpen(movementSync.getWorld(), movementSync.position.get())) {
                startFlight();
                return;
            }
            stage = Stage.LAND;
        }
        if (stage == Stage.PREPARE) {
            if (fly && movementSync.isGliding()) {
                startFlight();
                return;
            }
            if (!movementSync.onGround.get()) return;
            if (fly && isOpen(movementSync.getWorld(), movementSync.position.get())) {
                startFlight();
                return;
            }
            if (fly && approachTakeoff()) return;
            if (stopped || finished || !movementSync.isNavigationRequestCurrent(generation)) return;
            stage = Stage.LAND;
        }
        if (stage == Stage.LAND && movementSync.onGround.get() && !movementSync.isGliding()) {
            stage = Stage.WALK;
            List<PathStep> path = legs.plan(goal);
            if (stopped || finished || !movementSync.isNavigationRequestCurrent(generation)) return;
            if (path.isEmpty()) {
                movementSync.runIfNavigationRequestCurrent(generation, () -> {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.goto.no_path", goal.x, goal.y, goal.z));
                    movementSync.abandonNavigationRequest(generation);
                    finished = true;
                });
                return;
            }
            startChild(legs.walk(path, goal, false));
        }
    }

    private void startFlight() {
        MovementSync.getLogger().info("Travel: flight to {}", goal);
        stage = Stage.FLIGHT;
        startChild(legs.flight());
    }

    private void startChild(Movement movement) {
        MovementSync.INSTANCE.runIfNavigationRequestCurrent(generation, () -> {
            if (stopped || finished) return;
            child = movement;
            child.init();
        });
    }

    private List<PathStep> pathTo(Vector3i destination) {
        Vector3d position = MovementSync.INSTANCE.position.get();
        Node start = new Node((int) Math.floor(position.x), StandablePositionResolver.nodeY(position.y), (int) Math.floor(position.z));
        return new DStarLite(start, new Node(destination.x, destination.y, destination.z),
                MovementSync.INSTANCE.getWorld()).findPath(5000);
    }

    private boolean approachTakeoff() {
        MovementSync movementSync = MovementSync.INSTANCE;
        World world = movementSync.getWorld();
        Vector3d position = new Vector3d(movementSync.position.get());
        List<Vector3i> candidates = new ArrayList<>();
        int originX = (int) Math.floor(position.x);
        int originY = StandablePositionResolver.nodeY(position.y);
        int originZ = (int) Math.floor(position.z);
        // Bounded local search; unloaded chunks are never assumed to be empty sky.
        for (int x = originX - TAKEOFF_SEARCH_RADIUS; x <= originX + TAKEOFF_SEARCH_RADIUS; x++) {
            for (int z = originZ - TAKEOFF_SEARCH_RADIUS; z <= originZ + TAKEOFF_SEARCH_RADIUS; z++) {
                if (!world.chunkLoaded(x >> 4, z >> 4)) continue;
                for (int y = originY - 1; y <= originY + 1; y++) {
                    candidates.add(new Vector3i(x, y, z));
                }
            }
        }
        MovementSync.getLogger().info("Travel: checking {} takeoff candidates", candidates.size());
        candidates.sort(Comparator.comparingDouble(candidate -> position.distanceSquared(
                candidate.x + 0.5, candidate.y, candidate.z + 0.5)));
        int pathAttempts = 0;
        for (Vector3i candidate : candidates) {
            if (stopped || finished || !movementSync.isNavigationRequestCurrent(generation)) return false;
            var feet = StandablePositionResolver.feetY(world, candidate.x, candidate.y, candidate.z);
            if (feet.isEmpty() || !isOpen(world, new Vector3d(candidate.x + 0.5, feet.getAsDouble(), candidate.z + 0.5))) continue;
            // Path arrival permits a half-block offset; reserve that margin around the launch column.
            if (world.isBoxColliding(candidate.x - 1.5, feet.getAsDouble() + 0.01, candidate.z - 1.5,
                    candidate.x + 2.5, World.getMaxWorldY() + 1.0, candidate.z + 2.5)) continue;
            MovementSync.getLogger().info("Travel: planning takeoff approach to {}", candidate);
            // Each search expands up to 5000 nodes, so also bound total planning work.
            if (pathAttempts++ >= MAX_TAKEOFF_PATH_ATTEMPTS) return false;
            List<PathStep> path = legs.plan(candidate);
            if (stopped || finished || !movementSync.isNavigationRequestCurrent(generation)) return false;
            if (path.size() <= 1 || !path.get(path.size() - 1).getNode().equals(new Node(candidate.x, candidate.y, candidate.z))) continue;
            stage = Stage.APPROACH;
            startChild(legs.walk(path, candidate, true));
            return true;
        }
        return false;
    }

    public void requestRepath() {
        if (child instanceof PathMovement path) path.requestRepath();
    }

    public void requestRepath(int x, int y, int z) {
        if (child instanceof PathMovement path) path.requestRepath(x, y, z);
    }

    @Override
    public long getTime() { return -1; }

    @Override
    public void onStop() {
        stopped = true;
        finished = true;
        Movement currentChild = child;
        child = null;
        if (currentChild != null) currentChild.onStop();
    }
}
