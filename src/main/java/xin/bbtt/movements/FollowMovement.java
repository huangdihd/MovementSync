package xin.bbtt.movements;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.physics.EntityFlightTarget;
import xin.bbtt.physics.ElytraFlightController;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.pathfinding.*;

/** Owns follow mode changes; PathMovement retains all walking/replanning policy. */
public final class FollowMovement extends Movement {
    private final int entityId;
    private final double keepDistance;
    private final boolean allowDigging;
    private final long generation;
    private EntityFlightTarget target;
    private Movement child;
    private boolean flying;
    private boolean landing;
    private Vector3d landingAnchor;
    private Vector3d lostDestination;
    private boolean reportedLost;
    private boolean walkingPlanAttempted;
    private java.util.concurrent.CompletableFuture<LowAltitudeFlightPlan.Result> lowPlan;
    private int lowPlanRetryTicks;

    public FollowMovement(int entityId, double keepDistance, boolean allowDigging, long generation) {
        this.entityId = entityId;
        this.keepDistance = keepDistance > 0 ? keepDistance : 1;
        this.allowDigging = allowDigging;
        this.generation = generation;
    }

    @Override public void init() {
        var entity = MovementSync.INSTANCE.getWorld().getEntity(entityId);
        if (entity == null) { finished = true; return; }
        target = new EntityFlightTarget(entityId, keepDistance, entity.getPosition(), entity.getMovement());
        // Walking children validate the shared request identity as well as its
        // generation, including when this owner is initialized directly.
        MovementSync.INSTANCE.runIfNavigationRequestCurrent(generation,
                () -> MovementSync.INSTANCE.setFollowTargetId(entityId));
    }

    @Override public void onTick() {
        var sync = MovementSync.INSTANCE;
        if (finished || !sync.isNavigationRequestCurrent(generation)) { cancelLowPlan(); return; }
        if (lowPlanRetryTicks > 0) lowPlanRetryTicks--;
        // Reacquisition replaces the fixed search child and starts a fresh sample
        // window instead of interpreting the unseen displacement as target speed.
        if (target.lost() && sync.getWorld().getEntity(entityId) != null) {
            cancelLowPlan();
            target = new EntityFlightTarget(entityId, keepDistance,
                    sync.getWorld().getEntity(entityId).getPosition(),
                    sync.getWorld().getEntity(entityId).getMovement());
            reportedLost = false;
            lostDestination = null;
            landingAnchor = null;
            sync.setFollowTargetId(entityId);
            if (child != null) { child.onStop(); child = null; }
            landing = false;
            flying = false;
            walkingPlanAttempted = false;
        }
        target.sample(sync.getWorld());
        if (target.lost()) { searchLastPosition(); return; }
        // Touchdown precedes authoritative shared_flags clearing. Do not issue
        // walking input or a second deployment while that landing is pending.
        boolean water = ElytraFlyMovement.touchingWater();
        if (flying && child == null && (sync.onGround.get() || water)) {
            if (sync.isGliding() && !water) return;
            flying = false;
            landing = false;
            walkingPlanAttempted = false;
            MovementSync.getLogger().info("Follow: landed; flying -> walking at {}", sync.position.get());
            if (target.lost()) { finishLost(); return; }
        }
        if (target.lost() && !reportedLost) {
            reportedLost = true;
            MovementSync.getLogger().info("Follow: target lost; flying to last position {}", target.position());
        }
        // WalkingPhysics ground acceleration / (1 - ground drag), including
        // input damping. Same slipperiness and movement attributes as execution.
        double friction = xin.bbtt.physics.EnvironmentalPhysics.supportProperties(sync.getWorld(), sync.position.get()).slipperiness();
        double sprintSpeed = sync.getMovementParameters().movementSpeed(true) * 0.21600002
                / (friction * friction * friction) * 0.98 / (1 - friction * 0.91);
        boolean needsFlight = target.gliding() || target.pursuitSpeed() > sprintSpeed;
        if (water || target.lost() || !needsFlight) cancelLowPlan();
        if (!flying && !water && lowPlanRetryTicks == 0 && (sync.isGliding() || needsFlight && !target.lost()
                && sync.countHotbarFireworks() > 0
                && sync.getInventoryManager().canStartFlight(sync.position.get().distance(target.position())))) {
            var preparation = sync.getInventoryManager().prepareElytra(sync.onGround.get(), sync::queueBeforeMovement);
            if (sync.isGliding()) {
                cancelLowPlan();
                startFlight(null);
            } else if (preparation == xin.bbtt.inventory.InventoryManager.ElytraAction.READY) {
                // Ground pursuit uses an airborne endpoint; only stationary targets land.
                if (!TravelMovement.isOpen(sync.getWorld(), sync.position.get()) || target.landed()) {
                    if (prepareLowFlight()) return;
                } else startFlight(null);
            }
        }
        if (flying && landing && !target.lost() && needsFlight
                && (target.gliding() || landingAnchor != null
                && !target.landingWithin(landingAnchor, keepDistance + xin.bbtt.physics.PlayerPose.HALF_WIDTH))) {
            landing = false;
            replace(new ElytraFlyMovement(target, generation, Math.max(1, keepDistance)));
            MovementSync.getLogger().info("Follow: landing aborted; target moving again, resuming pursuit of {}", target.position());
        }
        if (flying && !landing && (target.lost() || target.landed() && !needsFlight
                && target.landingWithin(target.position(), keepDistance))) {
            try {
                // A supported target is a landing destination, not an invented launch
                // state. In a corridor the high-altitude reachability policy rejects
                // this floor; the local planner must prove it from our actual state.
                Vector3d site = ElytraFlightController.landingSite(sync.getWorld(),
                        target.position().x, target.position().z, target.position().y + .1);
                boolean local = site != null && !TravelMovement.isOpen(sync.getWorld(), sync.position.get());
                if (site == null) site = ElytraFlightController.selectForcedLandingTarget(sync.getWorld(),
                        new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), sync.fallDistance.get()),
                        sync.countHotbarFireworks(), sync.hasAttachedFirework());
                // A forced fallback may be safe but far from the entity. It is
                // not a follow arrival and must not replace a healthy pursuit.
                if (site != null && target.landingWithin(site, keepDistance)) {
                    landing = true;
                    landingAnchor = target.position();
                    walkingPlanAttempted = false;
                    var flight = new ElytraFlyMovement(site, generation, Math.max(1, keepDistance));
                    if (local) flight.withLowAltitudeLanding();
                    replace(flight);
                    MovementSync.getLogger().info("Follow: landing near target at {}", site);
                }
            } catch (IllegalStateException unavailable) {
                // Keep steering to the last known target while chunks become known.
            }
        }
        if (!flying && target.lost()) { finishLost(); return; }
        if (child == null && !flying && !walkingPlanAttempted) startWalking();
        if (child == null) return;
        child.onTick();
        if (child.isFinished()) {
            child.onStop();
            child = null;
            if (flying) lowPlanRetryTicks = 20;
            if (flying && (water || sync.onGround.get() && !sync.isGliding())) {
                flying = false;
                landing = false;
                walkingPlanAttempted = false;
                MovementSync.getLogger().info("Follow: landed; flying -> walking at {}", sync.position.get());
                if (target.lost()) finishLost();
            } else if (flying && !water && !sync.onGround.get()) {
                // A failed steering leg is not a touchdown. Retain an owner while
                // airborne, then resume walking after its safe landing.
                landing = true;
                landingAnchor = target.position();
                walkingPlanAttempted = false;
                var recovery = new ElytraFlyMovement(target.position(), generation, Math.max(1, keepDistance))
                        .forSafeLanding();
                if (!TravelMovement.isOpen(sync.getWorld(), sync.position.get())) recovery.withLowAltitudeLanding();
                replace(recovery);
            }
        }
    }

    /** Resolve an airborne last observation to support, without changing the observation itself. */
    private Vector3d searchDestination() {
        if (lostDestination == null) {
            var p = target.position();
            lostDestination = ElytraFlightController.landingSite(MovementSync.INSTANCE.getWorld(), p.x, p.z, p.y + .1);
            if (lostDestination == null) return p; // Unknown terrain: retain the search, not a false arrival.
        }
        return new Vector3d(lostDestination);
    }

    /** Loss is a fixed destination trip, never a moving-target pursuit route. */
    private void searchLastPosition() {
        var sync = MovementSync.INSTANCE;
        boolean water = ElytraFlyMovement.touchingWater();
        boolean grounded = water || sync.onGround.get() && !sync.isGliding();
        if (!reportedLost) {
            reportedLost = true;
            cancelLowPlan();
            if (child != null) { child.onStop(); child = null; }
            walkingPlanAttempted = false;
            // A failed pursuit's cooldown must not suppress this new trip.
            lowPlanRetryTicks = 0;
            flying = !grounded && sync.isGliding();
            landing = false;
            MovementSync.getLogger().info("Follow: entity {} absent from world tracking; last observation {}, landing search {}",
                    entityId, target.position(), searchDestination());
        } else if (flying && child == null && grounded) {
            // Legacy owner contract: explicitly relinquishing a landing child signals
            // completed touchdown. Normal child completion below resets flying and
            // checks distance, so early physical landings cannot enter this branch.
            finishLost();
            return;
        }
        if (grounded && sync.position.get().distance(searchDestination()) <= Math.max(1, keepDistance)) {
            if (child != null) { child.onStop(); child = null; }
            cancelLowPlan();
            finishLost();
            return;
        }
        if (child == null && flying) {
            var flight = new ElytraFlyMovement(searchDestination(), generation, Math.max(1, keepDistance));
            if (!TravelMovement.isOpen(sync.getWorld(), sync.position.get())) flight.withLowAltitudeLanding();
            landing = true;
            replace(flight);
        }
        if (!flying && !water && lowPlanRetryTicks == 0 && sync.countHotbarFireworks() > 0
                && sync.getInventoryManager().canStartFlight(sync.position.get().distance(searchDestination()))) {
            var preparation = sync.getInventoryManager().prepareElytra(sync.onGround.get(), sync::queueBeforeMovement);
            if (preparation == xin.bbtt.inventory.InventoryManager.ElytraAction.READY) {
                // Open rolling terrain also needs a feasible deployment/landing preview.
                if (prepareLowFlight()) return;
            }
        }
        if (child == null && !flying && !walkingPlanAttempted) startWalking();
        if (child == null) return;
        child.onTick();
        if (!child.isFinished()) return;
        child.onStop();
        child = null;
        if (flying && !grounded) {
            // Early flight failure still requires an airborne landing owner.
            var recovery = new ElytraFlyMovement(searchDestination(), generation, Math.max(1, keepDistance))
                    .forSafeLanding();
            if (!TravelMovement.isOpen(sync.getWorld(), sync.position.get())) recovery.withLowAltitudeLanding();
            replace(recovery);
        } else {
            flying = false;
            landing = false;
            walkingPlanAttempted = false;
            lowPlanRetryTicks = 20;
            // Child completion (including early touchdown) is not arrival.
            if (sync.position.get().distance(searchDestination()) <= Math.max(1, keepDistance)) finishLost();
        }
    }

    /** Worker geometry never runs under either the navigation or owner lock. */
    private boolean prepareLowFlight() {
        var sync = MovementSync.INSTANCE;
        if (lowPlanRetryTicks > 0) return false;
        boolean[] submitted = {false};
        sync.runIfNavigationRequestCurrent(generation, () -> {
            synchronized (this) {
                if (finished || lowPlan != null) return;
                // Hold the launch position while the worker previews deployment.
                if (child != null) { child.onStop(); child = null; }
                var state = new PhysicsState(sync.position.get(), sync.velocity.get(),
                        sync.onGround.get(), sync.fallDistance.get());
                var reserve = new xin.bbtt.physics.HazardRecovery.Context(
                        sync.getInventoryManager().getHotbarFireworkFlightDurations(), 0,
                        sync.hasAttachedFirework(), sync.getLandingMemory());
                lowPlan = !target.lost() && !target.gliding()
                        ? LowAltitudeFlightPlan.pursuit(sync.getWorld(), state, target.aim(state),
                                sync.getMovementParameters(), sync.isGliding(), reserve)
                        : LowAltitudeFlightPlan.submit(sync.getWorld(), state, target.lost() ? searchDestination() : target.aim(state),
                                sync.getMovementParameters(), sync.isGliding(), reserve);
                submitted[0] = true;
            }
        });
        if (submitted[0]) return true;
        sync.runIfNavigationRequestCurrent(generation, () -> {
            synchronized (this) {
                if (finished || lowPlan == null || !lowPlan.isDone()) return;
                var result = lowPlan.getNow(null);
                lowPlan = null;
                if (result != null && result.origin().distanceSquared(sync.position.get()) <= 1) {
                    startFlight(result);
                } else {
                    // Same twenty-tick cadence as flight corridor replanning.
                    lowPlanRetryTicks = 20;
                    walkingPlanAttempted = false;
                }
            }
        });
        synchronized (this) { return lowPlan != null; }
    }

    private void startFlight(LowAltitudeFlightPlan.Result result) {
        var sync = MovementSync.INSTANCE;
        sync.runIfNavigationRequestCurrent(generation, () -> {
            synchronized (this) {
                if (finished) return;
                var flight = target.lost()
                        ? new ElytraFlyMovement(searchDestination(), generation, Math.max(1, keepDistance))
                        : new ElytraFlyMovement(target, generation, Math.max(1, keepDistance));
                if (target.lost()) landing = true;
                if (result != null) flight.withLowAltitudePlan(result);
                flying = true;
                replace(flight);
                MovementSync.getLogger().info("Follow: walking -> flying; takeoff toward {}", target.position());
            }
        });
    }

    private synchronized void cancelLowPlan() {
        if (lowPlan != null) { lowPlan.cancel(false); lowPlan = null; walkingPlanAttempted = false; }
    }

    private void finishLost() {
        finished = true;
        MovementSync.getLogger().info("Follow: 跟丢了");
    }

    private void startWalking() {
        var sync = MovementSync.INSTANCE;
        Vector3d p = sync.position.get(), q = target.lost() ? searchDestination() : target.position();
        Node start = new Node((int)Math.floor(p.x), StandablePositionResolver.nodeY(p.y), (int)Math.floor(p.z));
        Node goal = new Node((int)Math.floor(q.x), StandablePositionResolver.nodeY(q.y), (int)Math.floor(q.z));
        var path = new DStarLite(start, goal, new DefaultPathfindingContext(sync.getWorld(), allowDigging)).findPath(2000);
        // Like planAndPublish, a failed/trivial initial walking plan is not retried.
        // Continue observing the target so that flight can still become feasible.
        walkingPlanAttempted = true;
        if (path.size() <= 1) return;
        if (target.lost()) {
            var destination = goal.toVector();
            // PathMovement validates global destination identity as well as generation.
            // Transfer the lost entity request to its fixed endpoint before ticking it.
            sync.runIfNavigationRequestCurrent(generation, () -> {
                sync.setFollowTargetId(-1);
                sync.setActiveGoal(destination);
                replace(new PathMovement(path, keepDistance, allowDigging, destination, -1, generation)
                        .asIntermediateLeg(destination));
            });
        } else replace(new PathMovement(path, keepDistance, allowDigging, null, entityId, generation));
    }

    private void replace(Movement next) {
        if (child != null) child.onStop();
        child = next;
        child.init();
    }
    @Override public long getTime() { return -1; }
    @Override public synchronized void onStop() {
        finished = true;
        cancelLowPlan();
        if (child != null) child.onStop();
    }
}
