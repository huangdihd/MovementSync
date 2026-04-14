package xin.bbtt.movements;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.pathfinding.Node;

import java.util.List;

public class PathMovement extends Movement {
    private final List<Node> path;
    private int currentIndex = 0;
    private int jumpTimer = 0;

    public PathMovement(List<Node> path) {
        this.path = path;
    }

    @Override
    public void init() {
        jumpTimer = 0;
        if (path == null || path.isEmpty()) {
            setFinished(true);
        }
    }

    @Override
    public void onTick() {
        if (currentIndex >= path.size()) {
            setFinished(true);
            return;
        }

        if (jumpTimer > 0) {
            jumpTimer--;
            return;
        }

        Node targetNode = path.get(currentIndex);
        Vector3d targetPos = new Vector3d(targetNode.x + 0.5, targetNode.y, targetNode.z + 0.5);
        Vector3d currentPos = MovementSync.Instance.position.get();

        double dx = Math.abs(currentPos.x - targetPos.x);
        double dz = Math.abs(currentPos.z - targetPos.z);
        double dy = Math.abs(currentPos.y - targetPos.y);

        if (dx < 0.3 && dz < 0.3 && dy < 0.5) {
            moveToNextNode();
            return;
        }
        applyHorizontalMovement(currentPos, targetPos, targetNode);
        
        MovementSync.Instance.lookAt(targetPos);
    }

    private void moveToNextNode() {
        currentIndex++;
        if (currentIndex < path.size()) return;
        setFinished(true);
        stopHorizontalMovement();
    }

    private boolean isPathDeviated(Vector3d currentPos, Node targetNode) {
        if (currentPos.y >= targetNode.y - 1.2) return false;

        MovementSync.Instance.getLogger().warn("Path deviation detected: fell below target height. Stopping.");
        setFinished(true);
        stopHorizontalMovement();
        return true;
    }

    private boolean isFallingUnexpectedly(Vector3d currentPos, Vector3d targetPos) {
        boolean onGround = MovementSync.Instance.onGround.get();
        double verticalDist = targetPos.y - currentPos.y;

        if (onGround || verticalDist < -0.5) return false;

        stopHorizontalMovement();
        return true;
    }

    private void applyHorizontalMovement(Vector3d currentPos, Vector3d targetPos, Node targetNode) {
        Vector3d direction = new Vector3d(targetPos).sub(currentPos);
        double verticalDist = targetPos.y - MovementSync.Instance.position.get().y;

        if (verticalDist > 0.5 && MovementSync.Instance.onGround.get()) {
            MovementSync.Instance.velocity.set(new Vector3d(0, 0.42, 0));
            jumpTimer = 10;
            return;
        }

        if (isPathDeviated(currentPos, targetNode)) return;
        if (isFallingUnexpectedly(currentPos, targetPos)) return;

        direction.y = 0;

        if (direction.lengthSquared() <= 0) {
            stopHorizontalMovement();
            return;
        }

        direction.normalize().mul(MovementSync.movementSpeed);

        Vector3d velocity = new Vector3d(direction.x, MovementSync.Instance.velocity.get().y, direction.z);
        MovementSync.Instance.velocity.set(velocity);
    }

    private void stopHorizontalMovement() {
        MovementSync.Instance.velocity.set(new Vector3d(0, MovementSync.Instance.velocity.get().y, 0));
    }

    @Override
    public long getTime() {
        return -1;
    }

    @Override
    public void onStop() {
        MovementSync.Instance.velocity.set(new Vector3d(0, 0, 0));
    }
}
