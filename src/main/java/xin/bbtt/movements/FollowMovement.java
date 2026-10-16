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
    private boolean reportedLost;
    private boolean walkingPlanAttempted;

    public FollowMovement(int entityId, double keepDistance, boolean allowDigging, long generation) {
        this.entityId = entityId;
        this.keepDistance = keepDistance > 0 ? keepDistance : 1;
        this.allowDigging = allowDigging;
        this.generation = generation;
    }

    @Override public void init() {
        var entity = MovementSync.INSTANCE.getWorld().getEntity(entityId);
        if (entity == null) { finished = true; return; }
        target = new EntityFlightTarget(entityId, keepDistance, entity.getPosition());
    }

    @Override public void onTick() {
        var sync = MovementSync.INSTANCE;
        if (finished || !sync.isNavigationRequestCurrent(generation)) return;
        target.sample(sync.getWorld());
        // Touchdown precedes authoritative shared_flags clearing. Do not issue
        // walking input or a second deployment while that landing is pending.
        boolean water = ElytraFlyMovement.touchingWater();
        if (flying && child == null && (sync.onGround.get() || water)) {
            if (sync.isGliding() && !water) return;
            flying = false;
            landing = false;
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
        boolean needsFlight = target.gliding() || target.horizontalSpeed() > sprintSpeed;
        if (!flying && !water && (sync.isGliding() || needsFlight && !target.lost()
                && sync.countHotbarFireworks() > 0
                && sync.getInventoryManager().canStartFlight(sync.position.get().distance(target.position())))) {
            var preparation = sync.getInventoryManager().prepareElytra(sync.onGround.get(), sync::queueBeforeMovement);
            if (sync.isGliding() || preparation == xin.bbtt.inventory.InventoryManager.ElytraAction.READY
                    && TravelMovement.isOpen(sync.getWorld(), sync.position.get())) {
                flying = true;
                replace(new ElytraFlyMovement(target, generation, Math.max(1, keepDistance)));
                MovementSync.getLogger().info("Follow: walking -> flying; takeoff toward {}", target.position());
            }
        }
        if (flying && landing && !target.lost() && needsFlight) {
            landing = false;
            replace(new ElytraFlyMovement(target, generation, Math.max(1, keepDistance)));
            MovementSync.getLogger().info("Follow: landing aborted; target moving again, resuming pursuit of {}", target.position());
        }
        if (flying && !landing && (target.lost() || target.landed() && !needsFlight)) {
            try {
                // Reuse the existing support search with the target as its centre.
                Vector3d site = new ElytraFlightController().selectForcedLandingTarget(sync.getWorld(),
                        new PhysicsState(target.position(), sync.velocity.get(), false),
                        sync.countHotbarFireworks(), sync.hasAttachedFirework());
                landing = true;
                walkingPlanAttempted = false;
                replace(new ElytraFlyMovement(site, generation, Math.max(1, keepDistance)));
                MovementSync.getLogger().info("Follow: landing near target at {}", site);
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
            if (flying && (water || sync.onGround.get() && !sync.isGliding())) {
                flying = false;
                landing = false;
                MovementSync.getLogger().info("Follow: landed; flying -> walking at {}", sync.position.get());
                if (target.lost()) finishLost();
            }
        }
    }

    private void finishLost() {
        finished = true;
        MovementSync.getLogger().info("Follow: 跟丢了");
    }

    private void startWalking() {
        var sync = MovementSync.INSTANCE;
        Vector3d p = sync.position.get(), q = target.position();
        Node start = new Node((int)Math.floor(p.x), StandablePositionResolver.nodeY(p.y), (int)Math.floor(p.z));
        Node goal = new Node((int)Math.floor(q.x), StandablePositionResolver.nodeY(q.y), (int)Math.floor(q.z));
        var path = new DStarLite(start, goal, new DefaultPathfindingContext(sync.getWorld(), allowDigging)).findPath(2000);
        // Like planAndPublish, a failed/trivial initial walking plan is not retried.
        // Continue observing the target so that flight can still become feasible.
        walkingPlanAttempted = true;
        if (path.size() <= 1) return;
        replace(new PathMovement(path, keepDistance, allowDigging, null, entityId, generation));
    }

    private void replace(Movement next) {
        if (child != null) child.onStop();
        child = next;
        child.init();
    }
    @Override public long getTime() { return -1; }
    @Override public void onStop() {
        finished = true;
        if (child != null) child.onStop();
    }
}
