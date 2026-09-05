package xin.bbtt.movements;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.StandablePositionResolver;

import java.util.List;

public class PathMovement extends Movement {
    /** Default follow keep radius in blocks. */
    private static final double DEFAULT_FOLLOW_KEEP_DISTANCE = 1.0;
    /** Adjacent path nodes should complete quickly; ten seconds means execution is livelocked. */
    private static final int MAX_TICKS_WITHOUT_NODE_PROGRESS = 200;

    private List<PathStep> path;
    private int currentIndex = 0;
    private volatile boolean repathRequested = false;
    private boolean immediateRepathRequested;
    private int repathThrottler = 0;
    /** Ticks since the last repath, to throttle update-driven repath bursts. */
    private int ticksSinceRepath = 100;
    /** Last position where we made horizontal progress, for stuck detection. */
    private Vector3d lastProgressPos = null;
    private int stuckTicks = 0;
    private final PathProgressWatchdog progressWatchdog = new PathProgressWatchdog(
            MAX_TICKS_WITHOUT_NODE_PROGRESS
    );
    /** While following, stay put as long as the target is within this horizontal distance (squared). */
    private final double followKeepDistanceSq;
    /** Immutable permission attached to this path and every internal replan. */
    private final boolean allowDigging;
    /** Static goal captured when this path request was created. */
    private final org.joml.Vector3i requestedGoal;
    /** Follow target captured when this path request was created, or -1. */
    private final int requestedFollowTargetId;
    /** Monotonic request identity; coordinates alone cannot distinguish replacements. */
    private final long requestGeneration;

    public PathMovement(List<PathStep> path) {
        this(path, DEFAULT_FOLLOW_KEEP_DISTANCE, false);
    }

    public PathMovement(List<PathStep> path, boolean allowDigging) {
        this(path, DEFAULT_FOLLOW_KEEP_DISTANCE, allowDigging);
    }

    public PathMovement(List<PathStep> path, double followKeepDistance) {
        this(path, followKeepDistance, false);
    }

    public PathMovement(List<PathStep> path, double followKeepDistance, boolean allowDigging) {
        this(
            path,
            followKeepDistance,
            allowDigging,
            goalFromPath(path),
            currentFollowTargetId(),
            currentNavigationGeneration()
        );
    }

    public PathMovement(
            List<PathStep> path,
            double followKeepDistance,
            boolean allowDigging,
            org.joml.Vector3i requestedGoal,
            int requestedFollowTargetId,
            long requestGeneration) {
        this.path = path;
        this.followKeepDistanceSq = followKeepDistance * followKeepDistance;
        this.allowDigging = allowDigging;
        this.requestedGoal = requestedGoal == null ? null : new org.joml.Vector3i(requestedGoal);
        this.requestedFollowTargetId = requestedFollowTargetId;
        this.requestGeneration = requestGeneration;
    }

    public void requestRepath() {
        this.repathRequested = true;
    }

    @Override
    public void init() {
        if (path == null || path.isEmpty()) {
            setFinished(true);
        }
    }

    @Override
    public void onTick() {
        if (!MovementSync.INSTANCE.runIfNavigationRequestCurrent(
                requestGeneration, this::onAuthorizedTick)) {
            setFinished(true);
            return;
        }
        if (immediateRepathRequested) {
            immediateRepathRequested = false;
            repathInternally();
        }
    }

    private void onAuthorizedTick() {
        if (isSuperseded()) {
            setFinished(true);
            return;
        }
        repathThrottler++;
        ticksSinceRepath++;
        // External repath requests come in bursts as the bot moves (block
        // updates, and chunks loading/unloading around it). Each repath rebuilds
        // the path from the current block, so honouring every one makes the
        // route jitter and step backwards a little. Throttle them, and defer
        // while airborne — rebuilding mid-air would drop the jump momentum and
        // drift the bot across a gap. Genuine path failures still replan
        // promptly through stuck detection and goal/placement handling.
        if (repathRequested && MovementSync.INSTANCE.onGround.get() && ticksSinceRepath >= 20) {
            repathRequested = false;
            scheduleRepath();
            return;
        }

        handleFollowTargetRepath();
        if (immediateRepathRequested) return;

        if (currentIndex >= path.size()) {
            finishPath();
            return;
        }

        PathStep currentStep = path.get(currentIndex);
        Node targetNode = currentStep.getNode();
        xin.bbtt.pathfinding.MovementType currentType = currentStep.getType();
        Vector3d targetPos = targetPosition(targetNode);
        Vector3d currentPos = MovementSync.INSTANCE.position.get();

        if (hasReachedNode(currentPos, targetPos)) {
            currentIndex++;
            progressWatchdog.nodeAdvanced();
            if (currentIndex >= path.size()) {
                finishPath();
                return;
            }
            currentStep = path.get(currentIndex);
            targetNode = currentStep.getNode();
            currentType = currentStep.getType();
            targetPos = targetPosition(targetNode);
        }

        if (requestedFollowTargetId == -1
                && requestedGoal != null
                && progressWatchdog.tick()) {
            abortLivelockedGoal(currentPos);
            return;
        }

        if (isPathDeviated(currentPos, targetNode)) return;
        if (isFallingUnexpectedly(currentPos, targetPos)) return;

        boolean isGapJump = currentType == BuiltinMovementType.GAP_JUMP;
        boolean onGround = MovementSync.INSTANCE.onGround.get();

        if (currentType == BuiltinMovementType.DIG
                && (!allowDigging || !MovementSync.INSTANCE.isNavigationRequestCurrent(requestGeneration))) {
            // The permission is path-scoped and immutable, but keep this guard
            // at the destructive dispatch boundary as a fail-closed invariant.
            markFinished();
            return;
        }

        if (currentType != null && (onGround || !currentType.requiresGroundToDispatch())) {
            Node prevNode = currentIndex > 0 ? path.get(currentIndex - 1).getNode() : new Node((int)Math.floor(currentPos.x), StandablePositionResolver.nodeY(currentPos.y), (int)Math.floor(currentPos.z));
            Movement customMovement = currentType.createMovement(prevNode, targetNode);
            if (customMovement != null) {
                if (customMovement instanceof NavigationBoundMovement navigationBound) {
                    navigationBound.bindNavigationRequest(requestGeneration);
                }
                if (!MovementSync.INSTANCE.insertMovementIfNavigationRequestCurrent(
                        requestGeneration, customMovement)) {
                    setFinished(true);
                }
                return;
            }
            if (!currentType.canWalkWhenNoMovement()) {
                // A placement move (bridge/pillar) couldn't run — typically the
                // hotbar ran out of blocks mid-path. Walking on would step into
                // the gap it was meant to fill, so replan: with an empty hotbar
                // the planner switches to no-placement strategies and routes
                // around it (or reports the goal unreachable) instead.
                scheduleRepath();
                return;
            }
        }

        // Stuck recovery: if the bot is on the ground with somewhere to go but
        // hasn't advanced for a while, the world likely changed since planning
        // (a block appeared, the ground vanished). Replan — but ONLY after a
        // real stall. A per-tick "is the target still walkable?" replan would
        // keep resetting the path to index 0, whose first target is the centre
        // of the bot's current block; mid-step that pulls the bot backwards,
        // producing a constant "backing up" stutter.
        if (!isGapJump && onGround && !MovementSync.INSTANCE.isRiding() && updateStuck(currentPos, targetPos)) {
            scheduleRepath();
            return;
        }

        applyPreciseMovement(currentPos, targetPos, isGapJump);
        updateLookDirection(currentPos, targetPos);
    }

    /**
     * Tracks horizontal progress toward the target and reports whether the bot
     * has been stalled long enough to warrant a replan. Returns false (and
     * keeps the counter reset) whenever the bot is essentially at the target or
     * is still making progress, so normal movement is never interrupted.
     */
    private boolean updateStuck(Vector3d currentPos, Vector3d targetPos) {
        double dx = targetPos.x - currentPos.x;
        double dz = targetPos.z - currentPos.z;
        boolean hasSomewhereToGo = (dx * dx + dz * dz) > 0.25;
        if (!hasSomewhereToGo) {
            stuckTicks = 0;
            lastProgressPos = new Vector3d(currentPos);
            return false;
        }

        double moved = lastProgressPos == null ? Double.MAX_VALUE
                : Math.pow(currentPos.x - lastProgressPos.x, 2) + Math.pow(currentPos.z - lastProgressPos.z, 2);
        if (moved > 0.0025) { // advanced more than 0.05 blocks since last checkpoint
            lastProgressPos = new Vector3d(currentPos);
            stuckTicks = 0;
            return false;
        }
        // ~2s of no meaningful progress (40 ticks @ 50ms).
        if (++stuckTicks > 40) {
            stuckTicks = 0;
            lastProgressPos = null;
            return true;
        }
        return false;
    }

    private void handleFollowTargetRepath() {
        if (requestedFollowTargetId == -1 || repathThrottler % 20 != 0) return;

        xin.bbtt.Entity.Entity entity = MovementSync.INSTANCE.getWorld().getEntity(requestedFollowTargetId);
        if (entity == null) return;

        Vector3d targetPos = entity.getPosition();
        if (isWithinFollowRange(targetPos)) return;

        PathStep lastStep = path.get(path.size() - 1);
        Node lastNode = lastStep.getNode();
        double distSq = Math.pow(targetPos.x - (lastNode.x + 0.5), 2) + Math.pow(targetPos.z - (lastNode.z + 0.5), 2);
        if (distSq > 4.0) {
            scheduleRepath();
        }
    }

    private boolean isWithinFollowRange(Vector3d targetPos) {
        Vector3d currentPos = MovementSync.INSTANCE.position.get();
        double distSq = Math.pow(currentPos.x - targetPos.x, 2) + Math.pow(currentPos.z - targetPos.z, 2);
        return distSq <= followKeepDistanceSq;
    }

    private void updateLookDirection(Vector3d currentPos, Vector3d targetPos) {
        Vector3d lookDiff = new Vector3d(targetPos).sub(currentPos);
        float targetYaw = (float) Math.toDegrees(Math.atan2(-lookDiff.x, lookDiff.z));
        targetYaw = (targetYaw + 180.0f) % 360.0f;
        MovementSync.INSTANCE.yaw.set(targetYaw);
        MovementSync.INSTANCE.pitch.set(0f);
    }

    private Vector3d targetPosition(Node node) {
        double feetY = xin.bbtt.pathfinding.StandablePositionResolver.feetY(
            MovementSync.INSTANCE.getWorld(), node.x, node.y, node.z
        ).orElse(node.y);
        return new Vector3d(node.x + 0.5, feetY, node.z + 0.5);
    }

    private boolean hasReachedNode(Vector3d currentPos, Vector3d targetPos) {
        double dx = Math.abs(currentPos.x - targetPos.x);
        double dz = Math.abs(currentPos.z - targetPos.z);
        double dy = Math.abs(currentPos.y - targetPos.y);
        boolean isLastNode = currentIndex == path.size() - 1;
        double horizontalTolerance = isLastNode ? 0.15 : 0.3;
        double verticalTolerance = 0.5;
        return dx < horizontalTolerance && dz < horizontalTolerance && dy < verticalTolerance;
    }

    private void applyPreciseMovement(Vector3d currentPos, Vector3d targetPos, boolean isGapJump) {
        Vector3d diff = new Vector3d(targetPos).sub(currentPos);
        double verticalDist = diff.y;
        diff.y = 0;
        double horizontalDistSq = diff.lengthSquared();
        
        if (horizontalDistSq <= 0.0001) {
            stopHorizontal();
            return;
        }

        double dist = Math.sqrt(horizontalDistSq);
        
        if (MovementSync.INSTANCE.isRiding()) {
            float yawRad = (float) Math.toRadians(MovementSync.INSTANCE.yaw.get());
            Vector3d forwardDir = new Vector3d(-Math.sin(yawRad), 0, Math.cos(yawRad));
            Vector3d sideDir = new Vector3d(-Math.cos(yawRad), 0, -Math.sin(yawRad));
            
            Vector3d moveDir = new Vector3d(diff).normalize();
            float forwardInput = (float) moveDir.dot(forwardDir);
            float sideInput = (float) moveDir.dot(sideDir);
            
            // Normalize inputs to -1..1 or similar if needed, here just basic projection
            MovementSync.INSTANCE.setRidingForward(forwardInput > 0.1 ? 1.0f : (forwardInput < -0.1 ? -1.0f : 0));
            MovementSync.INSTANCE.setRidingSideways(sideInput > 0.1 ? 1.0f : (sideInput < -0.1 ? -1.0f : 0));
            return;
        }

        double currentSpeed = MovementSync.movementSpeed;
        // Sprint during gap jumps: a diagonal landing is ~2.83 blocks away and
        // plain walking speed cannot carry the jump arc that far.
        if (isGapJump) currentSpeed *= MovementSync.gapJumpSpeedMultiplier;
        if (dist < 0.25) {
            currentSpeed *= (dist / 0.25);
            if (dist < 0.05) {
                MovementSync.INSTANCE.position.set(new Vector3d(targetPos.x, currentPos.y, targetPos.z));
                stopHorizontal();
                return;
            }
        }

        diff.normalize().mul(currentSpeed);
        MovementSync.INSTANCE.velocity.set(new Vector3d(diff.x, MovementSync.INSTANCE.velocity.get().y, diff.z));
        
        if (MovementSync.INSTANCE.onGround.get() && (verticalDist > 0.5 || isGapJump)) {
            MovementSync.INSTANCE.jump();
        }
    }

    private void finishPath() {
        if (requestedFollowTargetId != -1) {
            handleFollowFinish(requestedFollowTargetId);
            return;
        }

        if (requestedGoal != null) {
            handleGoalFinish(requestedGoal);
            return;
        }

        markFinished();
    }

    private void handleFollowFinish(int followTargetId) {
        xin.bbtt.Entity.Entity entity = MovementSync.INSTANCE.getWorld().getEntity(followTargetId);
        if (entity == null) { markFinished(); return; }

        Vector3d targetPos = entity.getPosition();
        if (!isWithinFollowRange(targetPos)) { scheduleRepath(); return; }

        stopHorizontal();
        MovementSync.INSTANCE.velocity.set(new Vector3d(0, 0, 0));
    }

    private void handleGoalFinish(org.joml.Vector3i goal) {
        Vector3d currentPos = MovementSync.INSTANCE.position.get();
        double dx = Math.abs(currentPos.x - (goal.x + 0.5)), dz = Math.abs(currentPos.z - (goal.z + 0.5)), dy = Math.abs(currentPos.y - goal.y);
        
        if (dx < 0.5 && dz < 0.5 && dy < 1.0) {
            markFinished();
        } else {
            scheduleRepath();
        }
    }

    private void markFinished() {
        setFinished(true);
        MovementSync.INSTANCE.runIfNavigationRequestCurrent(requestGeneration, () -> {
            stopHorizontal();
            MovementSync.INSTANCE.velocity.set(new Vector3d(0, 0, 0));
            if (requestedFollowTargetId == -1
                    && requestedGoal != null
                    && requestedGoal.equals(MovementSync.INSTANCE.getActiveGoal())) {
                MovementSync.INSTANCE.setActiveGoal(null);
            }
        });
    }

    private void abortLivelockedGoal(Vector3d currentPos) {
        MovementSync.getLogger().warn(
                "Pathfinding aborted: no path node completed for {} ticks despite replanning; position={}, goal={}",
                MAX_TICKS_WITHOUT_NODE_PROGRESS,
                currentPos,
                requestedGoal
        );
        markFinished();
    }

    private void scheduleRepath() {
        immediateRepathRequested = true;
    }

    private void repathInternally() {
        if (!MovementSync.INSTANCE.isNavigationRequestCurrent(requestGeneration)) {
            setFinished(true);
            return;
        }
        ticksSinceRepath = 0;
        org.joml.Vector3i targetNodePos = requestedGoal;
        if (requestedFollowTargetId != -1) {
            xin.bbtt.Entity.Entity entity = MovementSync.INSTANCE.getWorld().getEntity(requestedFollowTargetId);
            if (entity != null) {
                Vector3d p = entity.getPosition();
                targetNodePos = new org.joml.Vector3i(
                    (int)Math.floor(p.x), StandablePositionResolver.nodeY(p.y), (int)Math.floor(p.z));
            }
        }
        
        if (targetNodePos == null) { markFinished(); return; }
        
        Vector3d currentPos = MovementSync.INSTANCE.position.get();
        Node start = new Node((int)Math.floor(currentPos.x), StandablePositionResolver.nodeY(currentPos.y), (int)Math.floor(currentPos.z));
        Node goalNode = new Node(targetNodePos.x, targetNodePos.y, targetNodePos.z);
        
        if (start.equals(goalNode) && requestedFollowTargetId == -1) {
            markFinished();
            return;
        }

        xin.bbtt.pathfinding.DStarLite pathfinder = new xin.bbtt.pathfinding.DStarLite(
                start,
                goalNode,
                new xin.bbtt.pathfinding.DefaultPathfindingContext(
                        MovementSync.INSTANCE.getWorld(), allowDigging)
        );
        List<PathStep> newPath = pathfinder.findPath(2000);

        if (!MovementSync.INSTANCE.runIfNavigationRequestCurrent(
                requestGeneration, () -> applyRepathResult(newPath, start))) {
            setFinished(true);
        }
    }

    private void applyRepathResult(List<PathStep> newPath, Node start) {
        if (newPath != null && newPath.size() > 1) {
            this.path = newPath;
            // newPath[0] is the block we're already standing in. Targeting its
            // centre would pull the bot backwards whenever it has moved past
            // the centre, so head straight for the next node instead. This also
            // keeps a gap-jump edge at index 1 as the live target, so a repath
            // at the take-off block still launches the jump.
            this.currentIndex = 1;
            this.stuckTicks = 0;
            this.lastProgressPos = null;
        } else if (requestedFollowTargetId != -1) {
            stopHorizontal();
            MovementSync.INSTANCE.velocity.set(new Vector3d(0, 0, 0));
            this.path = List.of(new PathStep(start, BuiltinMovementType.WALK));
            this.currentIndex = 0;
        } else {
            markFinished();
        }
    }

    private boolean isSuperseded() {
        if (!MovementSync.INSTANCE.isNavigationRequestCurrent(requestGeneration)) return true;
        if (requestedFollowTargetId != -1) {
            return MovementSync.INSTANCE.getFollowTargetId() != requestedFollowTargetId;
        }
        return MovementSync.INSTANCE.getFollowTargetId() != -1
                || !java.util.Objects.equals(requestedGoal, MovementSync.INSTANCE.getActiveGoal());
    }

    private static org.joml.Vector3i goalFromPath(List<PathStep> path) {
        if (path == null || path.isEmpty()) return null;
        Node goal = path.get(path.size() - 1).getNode();
        return new org.joml.Vector3i(goal.x, goal.y, goal.z);
    }

    private static int currentFollowTargetId() {
        return MovementSync.INSTANCE == null ? -1 : MovementSync.INSTANCE.getFollowTargetId();
    }

    private static long currentNavigationGeneration() {
        return MovementSync.INSTANCE == null ? 0L : MovementSync.INSTANCE.getNavigationGeneration();
    }

    private void stopHorizontal() {
        Vector3d vel = MovementSync.INSTANCE.velocity.get();
        if (vel != null) MovementSync.INSTANCE.velocity.set(new Vector3d(0, vel.y, 0));
        MovementSync.INSTANCE.setRidingForward(0);
        MovementSync.INSTANCE.setRidingSideways(0);
    }

    private boolean isPathDeviated(Vector3d currentPos, Node targetNode) {
        if (MovementSync.INSTANCE.isRiding()) return false;
        if (currentPos.y >= targetNode.y - 1.2) return false;
        finishPath();
        return true;
    }

    private boolean isFallingUnexpectedly(Vector3d currentPos, Vector3d targetPos) {
        if (MovementSync.INSTANCE.isRiding()) return false;
        if (MovementSync.INSTANCE.onGround.get()) return false;
        if (MovementSync.INSTANCE.velocity.get().y > 0.01) return false;
        if (targetPos.y <= currentPos.y + 0.5) return false;
        stopHorizontal();
        return true;
    }

    @Override
    public long getTime() { return -1; }

    @Override
    public void onStop() { stopHorizontal(); }
}
