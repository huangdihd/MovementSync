package xin.bbtt.pathfinding;

import xin.bbtt.world.World;
import java.util.ArrayList;
import java.util.List;

/**
 * Strategy for standard walking, optionally digging through obstacles.
 */
public class WalkStrategy extends AbstractMovementStrategy {
    private final boolean allowDigging;

    public WalkStrategy() {
        this(false);
    }

    public WalkStrategy(boolean allowDigging) {
        this.allowDigging = allowDigging;
    }

    @Override
    public List<Edge> findEdges(Node u, World world) {
        List<Edge> edges = new ArrayList<>();
        int[] dx = {1, -1, 0, 0, 1, 1, -1, -1};
        int[] dz = {0, 0, 1, -1, 1, -1, 1, -1};

        for (int i = 0; i < 8; i++) {
            int nx = u.x + dx[i];
            int nz = u.z + dz[i];
            Node target = new Node(nx, u.y, nz);

            if (isStandable(nx, u.y, nz, world)
                    && StandablePositionResolver.canWalk(world, u, target)) {
                edges.add(new Edge(target, getEuclideanDistance(u, target), BuiltinMovementType.WALK));
            } else if (allowDigging && i < 4
                    && canDigThrough(nx, u.y, nz, world)
                    && (StandablePositionResolver.canWalkToSupport(world, u, target)
                        || StandablePositionResolver.canJumpToSupport(world, u, target))) {
                edges.add(new Edge(target, getEuclideanDistance(u, target) + 100.0, BuiltinMovementType.DIG));
            }
        }
        return edges;
    }
}
