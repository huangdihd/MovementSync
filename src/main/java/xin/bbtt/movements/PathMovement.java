package xin.bbtt.movements;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.pathfinding.Node;

import java.util.List;

public class PathMovement extends Movement {
    private List<Node> path;
    private int currentIndex = 0;
    private volatile boolean repathRequested = false;

    public PathMovement(List<Node> path) {
        this.path = path;
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
        if (repathRequested) {
            repathRequested = false;
            repathInternally();
        }

        if (currentIndex >= path.size()) {
            finishPath();
            return;
        }

        Node targetNode = path.get(currentIndex);
        Vector3d targetPos = new Vector3d(targetNode.x + 0.5, targetNode.y, targetNode.z + 0.5);
        Vector3d currentPos = MovementSync.Instance.position.get();

        // 1. Check if we reached the current node
        if (hasReachedNode(currentPos, targetPos)) {
            currentIndex++;
            if (currentIndex >= path.size()) {
                finishPath();
                return;
            }
            targetNode = path.get(currentIndex);
            targetPos = new Vector3d(targetNode.x + 0.5, targetNode.y, targetNode.z + 0.5);
        }

        // 2. Safety checks
        if (isPathDeviated(currentPos, targetNode)) return;
        if (isFallingUnexpectedly(currentPos, targetPos)) return;

        boolean isGapJump = false;
        if (currentIndex > 0) {
            Node prevNode = path.get(currentIndex - 1);
            double distSq = Math.pow(targetNode.x - prevNode.x, 2) + Math.pow(targetNode.z - prevNode.z, 2);
            if (distSq > 2.5) {
                isGapJump = true;
            }
        }

        // 3. Move towards the target
        applyPreciseMovement(currentPos, targetPos, isGapJump);
        
        // 4. Look in the direction of the target horizontally
        Vector3d lookDiff = new Vector3d(targetPos).sub(currentPos);
        float targetYaw = (float) Math.toDegrees(Math.atan2(-lookDiff.x, lookDiff.z));
        MovementSync.Instance.yaw.set(targetYaw);
        MovementSync.Instance.pitch.set(0f);
    }

    private boolean hasReachedNode(Vector3d currentPos, Vector3d targetPos) {
        double dx = Math.abs(currentPos.x - targetPos.x);
        double dz = Math.abs(currentPos.z - targetPos.z);
        double dy = Math.abs(currentPos.y - targetPos.y);

        boolean isLastNode = currentIndex == path.size() - 1;
        // Strict tolerance for ALL nodes to prevent cutting too much, 
        // but slightly looser for intermediate nodes to keep speed
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
        double currentSpeed = MovementSync.movementSpeed;

        // Braking: if we are close to the target, slow down to avoid orbiting
        if (dist < 0.25) {
            currentSpeed *= (dist / 0.25);
            // Snap to target if extremely close to stop oscillation
            if (dist < 0.05) {
                MovementSync.Instance.position.set(new Vector3d(targetPos.x, currentPos.y, targetPos.z));
                stopHorizontal();
                return;
            }
        }

        diff.normalize().mul(currentSpeed);
        MovementSync.Instance.velocity.set(new Vector3d(diff.x, MovementSync.Instance.velocity.get().y, diff.z));

        // Jump if needed
        if (MovementSync.Instance.onGround.get()) {
            if (verticalDist > 0.5) {
                MovementSync.Instance.jump();
            } else if (isGapJump) {
                MovementSync.Instance.jump();
            }
        }
    }

    private void finishPath() {
        org.joml.Vector3i goal = MovementSync.Instance.getActiveGoal();
        if (goal != null) {
            Vector3d currentPos = MovementSync.Instance.position.get();
            double dx = Math.abs(currentPos.x - (goal.x + 0.5));
            double dz = Math.abs(currentPos.z - (goal.z + 0.5));
            double dy = Math.abs(currentPos.y - goal.y);
            
            if (dx < 0.5 && dz < 0.5 && dy < 1.0) {
                setFinished(true);
                stopHorizontal();
                MovementSync.Instance.velocity.set(new Vector3d(0, 0, 0));
                MovementSync.Instance.setActiveGoal(null);
            } else {
                // Repath internally to continue towards the unreached goal!
                repathInternally();
            }
        } else {
            setFinished(true);
            stopHorizontal();
            MovementSync.Instance.velocity.set(new Vector3d(0, 0, 0));
            MovementSync.Instance.setActiveGoal(null);
        }
    }

    private void repathInternally() {
        org.joml.Vector3i targetNodePos = MovementSync.Instance.getActiveGoal();
        
        if (MovementSync.Instance.getFollowTargetId() != -1) {
            xin.bbtt.Entity.Entity entity = MovementSync.Instance.getWorld().getEntity(MovementSync.Instance.getFollowTargetId());
            if (entity != null) {
                Vector3d p = entity.getPosition();
                targetNodePos = new org.joml.Vector3i((int)Math.floor(p.x), (int)Math.floor(p.y), (int)Math.floor(p.z));
            }
        }

        if (targetNodePos == null) {
            setFinished(true);
            return;
        }

        Vector3d currentPos = MovementSync.Instance.position.get();
        Node start = new Node((int)Math.floor(currentPos.x), (int)Math.floor(currentPos.y), (int)Math.floor(currentPos.z));
        Node goalNode = new Node(targetNodePos.x, targetNodePos.y, targetNodePos.z);

        if (start.equals(goalNode) && MovementSync.Instance.getFollowTargetId() == -1) {
            setFinished(true);
            MovementSync.Instance.setActiveGoal(null);
            return;
        }

        xin.bbtt.pathfinding.DStarLite pathfinder = new xin.bbtt.pathfinding.DStarLite(start, goalNode, MovementSync.Instance.getWorld());
        List<Node> newPath = pathfinder.findPath(2000);

        if (newPath != null && newPath.size() > 1) {
            this.path = newPath;
            this.currentIndex = 0;
        } else {
            setFinished(true);
            stopHorizontal();
            if (MovementSync.Instance.getFollowTargetId() == -1) {
                MovementSync.Instance.setActiveGoal(null);
            }
        }
    }

    private void stopHorizontal() {
        Vector3d vel = MovementSync.Instance.velocity.get();
        MovementSync.Instance.velocity.set(new Vector3d(0, vel.y, 0));
    }

    private boolean isPathDeviated(Vector3d currentPos, Node targetNode) {
        if (currentPos.y >= targetNode.y - 1.2) return false;
        MovementSync.Instance.getLogger().warn("Path deviation: fell below target. Stopping.");
        finishPath();
        return true;
    }

    private boolean isFallingUnexpectedly(Vector3d currentPos, Vector3d targetPos) {
        if (MovementSync.Instance.onGround.get()) return false;
        if (MovementSync.Instance.velocity.get().y > 0.01) return false;
        if (targetPos.y > currentPos.y + 0.5) {
            stopHorizontal();
            return true;
        }
        return false;
    }

    @Override
    public long getTime() { return -1; }

    @Override
    public void onStop() { stopHorizontal(); }
}
