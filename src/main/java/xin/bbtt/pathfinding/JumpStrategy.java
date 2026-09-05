package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import xin.bbtt.world.World;
import java.util.ArrayList;
import java.util.List;

/**
 * Strategy for jumping up by one block, optionally digging the target cell.
 */
public class JumpStrategy extends AbstractMovementStrategy {
    private final boolean allowDigging;

    public JumpStrategy() {
        this(false);
    }

    public JumpStrategy(boolean allowDigging) {
        this.allowDigging = allowDigging;
    }

    @Override
    public List<Edge> findEdges(Node u, World world) {
        List<Edge> edges = new ArrayList<>();
        int[] dx = {1, -1, 0, 0};
        int[] dz = {0, 0, 1, -1};

        for (int i = 0; i < 4; i++) {
            int nx = u.x + dx[i];
            int nz = u.z + dz[i];

            // Exact collision tops can require a jump while still mapping to the
            // same integer nodeY (for example carpet 0.0625 -> block top 1.0).
            Node sameYTarget = new Node(nx, u.y, nz);
            if (isStandable(nx, u.y, nz, world)
                    && StandablePositionResolver.canJump(world, u, sameYTarget)
                    && !StandablePositionResolver.canWalk(world, u, sameYTarget)) {
                edges.add(new Edge(
                    sameYTarget,
                    getEuclideanDistance(u, sameYTarget) + 0.5,
                    BuiltinMovementType.JUMP
                ));
            }

            Node raisedTarget = new Node(nx, u.y + 1, nz);

            // Can only jump if head space is clear
            if (world.isPassable(new Vector3d(u.x, u.y + 2, u.z))) {
                if (isStandable(nx, u.y + 1, nz, world)
                        && StandablePositionResolver.canJump(world, u, raisedTarget)) {
                    edges.add(new Edge(raisedTarget, getEuclideanDistance(u, raisedTarget) + 0.5, BuiltinMovementType.JUMP));
                } else if (allowDigging
                        && canDigThrough(nx, u.y + 1, nz, world)
                        && StandablePositionResolver.canJumpToSupport(world, u, raisedTarget)) {
                    edges.add(new Edge(raisedTarget, getEuclideanDistance(u, raisedTarget) + 100.5, BuiltinMovementType.DIG));
                }
            }
        }
        return edges;
    }
}
