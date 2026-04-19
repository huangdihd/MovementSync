package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import xin.bbtt.world.World;
import java.util.ArrayList;
import java.util.List;

/**
 * Strategy for falling down from a height.
 */
public class FallStrategy extends AbstractMovementStrategy {
    private final int fallDownHeight;

    public FallStrategy(int fallDownHeight) {
        this.fallDownHeight = fallDownHeight;
    }

    @Override
    public List<Edge> findEdges(Node u, World world) {
        List<Edge> edges = new ArrayList<>();
        int[] dx = {1, -1, 0, 0};
        int[] dz = {0, 0, 1, -1};

        for (int i = 0; i < 4; i++) {
            int nx = u.x + dx[i];
            int nz = u.z + dz[i];

            for (int dy = -1; dy >= -this.fallDownHeight; dy--) {
                if (isStandable(nx, u.y + dy, nz, world)) {
                    Node target = new Node(nx, u.y + dy, nz);
                    edges.add(new Edge(target, getEuclideanDistance(u, target) + (u.y - target.y) * 2.0));
                    break;
                }
                // Stop if blocked by non-passable block
                if (!world.isPassable(new Vector3d(nx, u.y + dy, nz))) break; 
            }
        }
        return edges;
    }
}
