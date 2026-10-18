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
        default xin.bbtt.inventory.InventoryManager.ElytraAction prepareFlight() {
            return xin.bbtt.inventory.InventoryManager.ElytraAction.READY;
        }
        default boolean hasFlightBudget() { return true; }
    }

    private final Legs legs;
    private final xin.bbtt.flight.FlightConfiguration flightConfiguration;
    private enum Stage { PREPARE, APPROACH, TAKEOFF, FLIGHT, LAND, WALK }
    private final Vector3i goal;
    private final long generation;
    private final boolean fly;
    private Stage stage = Stage.PREPARE;
    private volatile Movement child;
    private volatile boolean stopped;
    private boolean takeoffForLanding;
    private LowAltitudeFlightPlan.PlanFuture lowPlan;
    private String lowFlightFailure;
    private xin.bbtt.physics.PlayerInput waitingInput;

    /** No child owns the controls while preparation/planning runs. Sneak also brakes at ledges. */
    private void holdForPlanning(MovementSync sync) {
        sync.runIfNavigationRequestCurrent(generation, () -> {
            if (stopped || finished || child != null) return;
            sync.clearInput(); // Also discard a queued one-shot jump from the previous leg.
            waitingInput = new xin.bbtt.physics.PlayerInput(false, false, false, false,
                    false, sync.onGround.get(), false);
            sync.setInput(waitingInput);
        });
    }

    private void releasePlanningInput(MovementSync sync) {
        GroundedFlightStop.release(sync, waitingInput);
        waitingInput = null;
    }

    String lowFlightFailure() { return lowFlightFailure; }

    private boolean rejectLowFlight(String reason) {
        lowFlightFailure = reason;
        MovementSync.INSTANCE.runIfNavigationRequestCurrent(generation, () ->
                MovementSync.getLogger().info("Travel: under-roof flight preparation failed: {}; falling back to walking", reason));
        return false;
    }

    /** Flight proximity is not arrival: the final ground leg must reach the requested elevation. */
    private boolean atSupportedGoal(MovementSync sync) {
        Vector3d position = sync.position.get();
        boolean supported = sync.onGround.get()
                || xin.bbtt.pathfinding.EnvironmentalTraversal.inWater(sync.getWorld(), position)
                || xin.bbtt.pathfinding.EnvironmentalTraversal.climbing(sync.getWorld(), position);
        return supported && !sync.isGliding()
                && Math.abs(position.x - (goal.x + .5)) < .5
                && Math.abs(position.z - (goal.z + .5)) < .5
                && Math.abs(position.y - goal.y) < 1;
    }

    private void abortTravel(MovementSync sync, String reason) {
        sync.runIfNavigationRequestCurrent(generation, () -> {
            MovementSync.getLogger().info("Travel: navigation aborted: {}", reason);
            sync.abandonNavigationRequest(generation, reason);
            releasePlanningInput(MovementSync.INSTANCE);
            finished = true;
        });
    }

    public TravelMovement(Vector3i goal, long generation, boolean fly) {
        this(goal, generation, fly, null, new xin.bbtt.flight.FlightContextBuilder().addStandard());
    }

    public TravelMovement(Vector3i goal, long generation, boolean fly,
                          xin.bbtt.flight.FlightContextBuilder builder) {
        this(goal, generation, fly, null, builder);
    }

    TravelMovement(Vector3i goal, long generation, boolean fly, Legs legs) {
        this(goal, generation, fly, legs, new xin.bbtt.flight.FlightContextBuilder().addStandard());
    }

    TravelMovement(Vector3i goal, long generation, boolean fly, Legs legs,
                   xin.bbtt.flight.FlightContextBuilder builder) {
        this.flightConfiguration = builder.build();
        this.goal = new Vector3i(goal);
        this.generation = generation;
        this.fly = fly;
        this.legs = legs == null ? new Legs() {
            @Override public List<PathStep> plan(Vector3i destination) { return pathTo(destination); }
            @Override public Movement walk(List<PathStep> path, Vector3i destination, boolean intermediate) {
                PathMovement movement = new PathMovement(path, -1, false, destination, -1, generation)
                        .yieldingUnexpectedFalls();
                return intermediate ? movement.asIntermediateLeg(TravelMovement.this.goal) : movement;
            }
            @Override public xin.bbtt.inventory.InventoryManager.ElytraAction prepareFlight() {
                return VanillaGlideLifecycle.prepareFlight(MovementSync.INSTANCE);
            }
            @Override public boolean hasFlightBudget() {
                return VanillaGlideLifecycle.hasFlightBudget(MovementSync.INSTANCE, TravelMovement.this.goal);
            }
            @Override public Movement flight() {
                Vector3i destination = TravelMovement.this.goal;
                var snapshot = new xin.bbtt.flight.FlightContextBuilder()
                        .cruiseDamageFirst(flightConfiguration.cruiseDamageFirst());
                flightConfiguration.modes().forEach(snapshot::addMode);
                flightConfiguration.costs().forEach(snapshot::addCostTerm);
                flightConfiguration.damageRules().forEach(snapshot::addDamageRule);
                return new ElytraFlyMovement(new Vector3d(destination.x + 0.5, destination.y, destination.z + 0.5),
                        generation, snapshot);
            }
        } : legs;
    }

    public static boolean shouldFly(double distance, boolean elytra, boolean fireworks,
                                    boolean walk, boolean gliding) {
        return VanillaGlideLifecycle.shouldFly(distance, elytra, fireworks, walk, gliding);
    }

    public static boolean shouldFly(double distance, xin.bbtt.physics.ElytraDurability durability,
                                    boolean fireworks, boolean walk, boolean gliding) {
        return VanillaGlideLifecycle.shouldFly(distance, durability, fireworks, walk, gliding);
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
    public void init() { holdForPlanning(MovementSync.INSTANCE); }

    @Override
    public void onTick() {
        // Not under the navigation lock: path planning and flight previews here can be slow,
        // and cancellation must not wait for them. Children guard their own writes.
        MovementSync movementSync = MovementSync.INSTANCE;
        if (stopped || finished || !movementSync.isNavigationRequestCurrent(generation)) return;
        Movement currentChild = child;
        if (currentChild != null) {
            releasePlanningInput(movementSync);
            currentChild.onTick();
            if (!currentChild.isFinished()) return;
            child = null;
            currentChild.onStop();
            if (stopped || finished || !movementSync.isNavigationRequestCurrent(generation)) return;
            if (currentChild instanceof ElytraFlyMovement flight && flight.completedAboveRoof()) {
                movementSync.runIfNavigationRequestCurrent(generation, () -> {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.goto.target_below_roof"));
                    movementSync.abandonNavigationRequest(generation, "target below roof is unreachable from roof landing");
                    releasePlanningInput(MovementSync.INSTANCE);
            finished = true;
                });
                return;
            }
            if (currentChild instanceof PathMovement path
                    && path.completion() == PathMovement.Completion.FAILED) {
                abortTravel(movementSync, "walking leg failed: " + path.failureReason());
                return;
            }
            if (currentChild instanceof PathMovement path
                    && path.completion() == PathMovement.Completion.UNEXPECTED_FALL) {
                stage = Stage.PREPARE;
            } else if (stage == Stage.WALK) {
                if (atSupportedGoal(movementSync)) {
                    movementSync.runIfNavigationRequestCurrent(generation, () -> {
                        movementSync.setActiveGoal(null);
                        releasePlanningInput(MovementSync.INSTANCE);
            finished = true;
                    });
                } else abortTravel(movementSync, "walking leg ended before supported 3D arrival");
                return;
            }
            holdForPlanning(movementSync);
            if (stage == Stage.TAKEOFF) {
                stage = takeoffForLanding ? Stage.LAND : Stage.FLIGHT;
                startChild(takeoffForLanding ? extendedLanding() : legs.flight());
                return;
            }
            if (stage == Stage.APPROACH && isOpen(movementSync.getWorld(), movementSync.position.get())) {
                startFlight();
                return;
            }
            if (currentChild instanceof ElytraFlyMovement && !atSupportedGoal(movementSync)
                    && xin.bbtt.listeners.RegistryDataListener.hasCeiling()
                    && !isOpen(movementSync.getWorld(), movementSync.position.get())
                    && movementSync.countHotbarFireworks() == 0) {
                rejectLowFlight("insufficient fireworks to continue; landed on known support");
            }
            if (stage != Stage.PREPARE) stage = Stage.LAND;
        }
        holdForPlanning(movementSync);
        boolean environmental = xin.bbtt.pathfinding.EnvironmentalTraversal.inWater(
                movementSync.getWorld(), movementSync.position.get())
                || xin.bbtt.pathfinding.EnvironmentalTraversal.climbing(
                        movementSync.getWorld(), movementSync.position.get());
        boolean belowRoof = xin.bbtt.listeners.RegistryDataListener.hasCeiling()
                && !isOpen(movementSync.getWorld(), movementSync.position.get());
        if (belowRoof && stage == Stage.PREPARE) {
            if (fly && !environmental && prepareLowFlight(movementSync)) return;
            stage = Stage.LAND;
            if (!movementSync.onGround.get() && !environmental) {
                startChild(extendedLanding());
                return;
            }
        }
        if (!belowRoof && !flightConfiguration.isLegacy()) {
            // Still compare the real vanilla preview when plugins are registered. If all
            // plugins decline transitions, retain legacy preparation/approach behavior.
            if (hasPluginTransition()) {
                if (tickExtended(movementSync, environmental)) return;
            } else if (stage == Stage.PREPARE && (fly || !movementSync.onGround.get())
                    && startTransition(xin.bbtt.flight.FlightTransitions.Kind.TAKEOFF, !fly)) return;
        }
        if (stage == Stage.PREPARE) {
            if (fly && movementSync.isGliding()) {
                if (belowRoof) {
                    stage = Stage.LAND;
                    startChild(extendedLanding());
                    return;
                }
                startFlight();
                return;
            }
            if (!movementSync.onGround.get() && !environmental) {
                if (VanillaGlideLifecycle.eligibleForAirborneTakeoff(movementSync)) {
                    boolean cruise = fly && !belowRoof && legs.hasFlightBudget();
                    ElytraFlyMovement flight = new ElytraFlyMovement(
                            new Vector3d(goal.x + 0.5, goal.y, goal.z + 0.5), generation);
                    stage = Stage.FLIGHT;
                    MovementSync.getLogger().info("Travel: airborne takeoff, cruise={}", cruise);
                    startChild(cruise ? flight : flight.forSafeLanding());
                }
                return;
            }
            if (fly && !environmental && isOpen(movementSync.getWorld(), movementSync.position.get())) {
                startFlight();
                return;
            }
            if (fly && !environmental && approachTakeoff()) return;
            if (stopped || finished || !movementSync.isNavigationRequestCurrent(generation)) return;
            stage = Stage.LAND;
        }
        if (stage == Stage.LAND && !environmental && movementSync.onGround.get()
                && ElytraFlyMovement.hazardousGroundHandoff(movementSync)
                && (movementSync.isGliding() || movementSync.velocity.get().x != 0
                    || movementSync.velocity.get().z != 0)
                && !GroundedFlightStop.stopped(movementSync)) {
            // Keep flight control alive while metadata still selects glide physics.
            if (movementSync.isGliding()) startChild(extendedLanding());
            return;
        }
        if (stage == Stage.LAND && (ElytraFlyMovement.touchingWater()
                || (movementSync.onGround.get() || environmental) && !movementSync.isGliding())) {
            stage = Stage.WALK;
            List<PathStep> path = legs.plan(goal);
            if (stopped || finished || !movementSync.isNavigationRequestCurrent(generation)) return;
            if (path.isEmpty()) {
                movementSync.runIfNavigationRequestCurrent(generation, () -> {
                    MovementSync.getLogger().info(LangManager.get("movementsync.command.goto.no_path", goal.x, goal.y, goal.z));
                    movementSync.abandonNavigationRequest(generation, "no walking path to destination");
                    releasePlanningInput(MovementSync.INSTANCE);
            finished = true;
                });
                return;
            }
            startChild(legs.walk(path, goal, false));
        }
    }

    /** Planning never owns navigation locks; publication rechecks the owning generation. */
    private boolean prepareLowFlight(MovementSync sync) {
        // A walking interruption must not retry a rejected edge launch in this request.
        if (lowFlightFailure != null) return false;
        holdForPlanning(sync);
        if (flightConfiguration.modes().stream().noneMatch(mode -> mode instanceof xin.bbtt.flight.VanillaGlideMode))
            return rejectLowFlight("vanilla glide mode disabled");
        if (!LowAltitudeFlightPlan.admissible(sync.position.get(), new Vector3d(goal.x + .5, goal.y, goal.z + .5)))
            return rejectLowFlight("invalid route coordinates");
        var equipment = legs.prepareFlight();
        if (equipment == xin.bbtt.inventory.InventoryManager.ElytraAction.WAITING) return true;
        if (equipment == xin.bbtt.inventory.InventoryManager.ElytraAction.LAND || !sync.hasEquippedElytra())
            return rejectLowFlight("equipment unavailable or unsafe");
        if (!legs.hasFlightBudget()) return rejectLowFlight("insufficient flight equipment budget");
        if (lowPlan == null) {
            lowPlan = LowAltitudeFlightPlan.submit(sync.getWorld(),
                    new xin.bbtt.physics.PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), sync.fallDistance.get()),
                    new Vector3d(goal.x + .5, goal.y, goal.z + .5), sync.getMovementParameters(), sync.isGliding(),
                    new xin.bbtt.physics.HazardRecovery.Context(sync.inventoryManager.getHotbarFireworkFlightDurations(),
                            0, sync.hasAttachedFirework(), sync.getLandingMemory()));
            return true;
        }
        if (!lowPlan.isDone()) return true;
        var result = lowPlan.getNow(null);
        var status = lowPlan.status();
        var detail = lowPlan.detail();
        lowPlan = null;
        if (result == null) return rejectLowFlight(detail == null ? lowPlanFailure(status) : detail);
        if (result.origin().distanceSquared(sync.position.get()) > .25)
            return rejectLowFlight("launch position changed during planning");
        if (result.destination().distanceSquared(new Vector3d(goal.x + .5, goal.y, goal.z + .5)) > .01)
            return rejectLowFlight("route destination changed");
        sync.runIfNavigationRequestCurrent(generation, () -> {
            if (stopped || finished) return;
            var movement = legs.flight();
            if (!(movement instanceof ElytraFlyMovement flight)) return;
            flight.withLowAltitudePlan(result);
            stage = Stage.FLIGHT;
            startChild(flight);
        });
        return child != null;
    }

    static String lowPlanFailure(LowAltitudeFlightPlan.Status status) {
        return switch (status) {
            case DEPLOYMENT_REJECTED -> "deployment blocked or unsafe";
            case NO_CORRIDOR -> "no safe corridor within geometric search budget";
            case INVALID_COORDINATES -> "invalid route coordinates";
            case BUSY -> "flight planner busy";
            case CANCELLED -> "flight planning cancelled";
            case FAILED -> "flight planner failed";
            default -> "flight planner returned no route (" + status + ")";
        };
    }

    /** Returns false to hand this tick to the vanilla preparation below. */
    private boolean tickExtended(MovementSync sync, boolean environmental) {
        boolean supported = sync.onGround.get() || environmental;
        if (stage == Stage.PREPARE) {
            if ((fly || !supported) && startTransition(xin.bbtt.flight.FlightTransitions.Kind.TAKEOFF, !fly)) return true;
            // The vanilla preview needs open sky here. With no winner, an equipped elytra still
            // gets the vanilla walk to an open launch spot instead of walking the whole route.
            if (fly && supported && !environmental && sync.hasEquippedElytra()) return false;
            stage = Stage.LAND;
        }
        if (stage == Stage.LAND && !supported) {
            if (!startTransition(xin.bbtt.flight.FlightTransitions.Kind.LANDING, true)) {
                // No transition is not permission to disable flight: retain mode-owned airborne control.
                stage = Stage.LAND;
                startChild(extendedLanding());
            }
            return true;
        }
        if (stage == Stage.LAND && supported) {
            if (!environmental && ElytraFlyMovement.hazardousGroundHandoff(sync)
                    && !GroundedFlightStop.stopped(sync)) {
                if (sync.isGliding()) startChild(extendedLanding());
                return true;
            }
            stage = Stage.WALK;
            List<PathStep> path = legs.plan(goal);
            if (stopped || finished || !sync.isNavigationRequestCurrent(generation)) return true;
            if (path.isEmpty()) {
                sync.abandonNavigationRequest(generation, "no walking path to destination");
                releasePlanningInput(MovementSync.INSTANCE);
            finished = true;
            } else startChild(legs.walk(path, goal, false));
        }
        return true;
    }

    private Movement extendedLanding() {
        Movement movement = legs.flight();
        return movement instanceof ElytraFlyMovement flight ? flight.forSafeLanding() : movement;
    }

    private boolean hasPluginTransition() {
        var context = transitionContext(!fly);
        for (var mode : flightConfiguration.modes()) {
            if (mode instanceof xin.bbtt.flight.VanillaGlideMode || !mode.isAvailable(context)) continue;
            if (mode.previewTakeoff(context) != null || mode.previewLanding(context) != null) return true;
        }
        return false;
    }

    private xin.bbtt.flight.FlightContext transitionContext(boolean emergency) {
        MovementSync sync = MovementSync.INSTANCE;
        var inventory = new java.util.HashMap<String, Integer>();
        var stacks = sync.inventoryManager.getInventory();
        if (stacks != null) for (var stack : stacks) {
            if (stack == null) continue;
            var item = xin.bbtt.inventory.ItemRegistry.Instance.getItem(stack.getId());
            if (item != null) inventory.merge("minecraft:" + item.getName(), stack.getAmount(), Integer::sum);
        }
        return new xin.bbtt.flight.FlightContext(sync.getWorld(),
                new xin.bbtt.physics.PhysicsState(sync.position.get(), sync.velocity.get(),
                        sync.onGround.get(), sync.fallDistance.get()), sync.getMovementParameters(),
                xin.bbtt.physics.DamageParameters.fromPlayer(sync.inventoryManager,
                        sync.getMovementParameterState().effectsSnapshot()),
                sync.getMovementParameterState().health(), new Vector3d(goal.x + .5, goal.y, goal.z + .5),
                sync.yaw.get(), sync.pitch.get(), sync.countHotbarFireworks(), sync.hasAttachedFirework(),
                ElytraFlyMovement.DEFAULT_ARRIVAL_RADIUS, emergency, inventory);
    }

    private boolean startTransition(xin.bbtt.flight.FlightTransitions.Kind kind, boolean emergency) {
        var context = transitionContext(emergency);
        var selected = xin.bbtt.flight.FlightTransitions.select(flightConfiguration, context, kind);
        if (selected == null) return false;
        takeoffForLanding = emergency || selected.mode() instanceof xin.bbtt.flight.VanillaGlideMode
                && !MovementSync.INSTANCE.onGround.get() && !legs.hasFlightBudget();
        stage = kind == xin.bbtt.flight.FlightTransitions.Kind.TAKEOFF ? Stage.TAKEOFF : Stage.LAND;
        startChild(selected.movement());
        return true;
    }

    private void startFlight() {
        MovementSync sync = MovementSync.INSTANCE;
        if (!sync.isGliding()) {
            var preparation = legs.prepareFlight();
            if (preparation == xin.bbtt.inventory.InventoryManager.ElytraAction.WAITING) {
                stage = Stage.PREPARE;
                return;
            }
            if (preparation == xin.bbtt.inventory.InventoryManager.ElytraAction.LAND || !legs.hasFlightBudget()) {
                stage = Stage.LAND;
                return;
            }
        }
        MovementSync.getLogger().info("Travel: flight to {}", goal);
        stage = Stage.FLIGHT;
        startChild(legs.flight());
    }

    private void startChild(Movement movement) {
        MovementSync.INSTANCE.runIfNavigationRequestCurrent(generation, () -> {
            if (stopped || finished) return;
            // Even the final walking leg is subordinate: only this owner may clear
            // the static goal, after observing supported, non-gliding 3D arrival.
            if (movement instanceof PathMovement path) path.asIntermediateLeg(goal);
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
        if (lowPlan != null) lowPlan.cancel(false);
        Movement currentChild = child;
        child = null;
        if (currentChild != null) currentChild.onStop();
        releasePlanningInput(MovementSync.INSTANCE);
    }
}
