package xin.bbtt.movements;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.pathfinding.Node;

import java.util.List;

public class PathMovement extends Movement {
    private final List<Node> path;
    private int currentIndex = 0;

    public PathMovement(List<Node> path) {
        this.path = path;
    }

    @Override
    public void init() {
        if (path == null || path.isEmpty()) {
            setFinished(true);
        }
    }

    @Override
    public void onTick() {
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

        // 3. Move towards the target
        applyPreciseMovement(currentPos, targetPos);
        
        // 4. Look at the current target
        MovementSync.Instance.directLookAt(targetPos);
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

    private void applyPreciseMovement(Vector3d currentPos, Vector3d targetPos) {
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
        if (verticalDist > 0.5 && MovementSync.Instance.onGround.get()) {
            MovementSync.Instance.jump();
        }
    }

    private void finishPath() {
        setFinished(true);
        stopHorizontal();
        MovementSync.Instance.velocity.set(new Vector3d(0, 0, 0));
        MovementSync.Instance.setActiveGoal(null);
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
