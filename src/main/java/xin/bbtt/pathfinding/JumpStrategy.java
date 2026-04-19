package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import xin.bbtt.world.World;
import java.util.ArrayList;
import java.util.List;

/**
 * Strategy for jumping up by one block.
 */
public class JumpStrategy extends AbstractMovementStrategy {
    @Override
    public List<Edge> findEdges(Node u, World world) {
        List<Edge> edges = new ArrayList<>();
        int[] dx = {1, -1, 0, 0};
        int[] dz = {0, 0, 1, -1};

        for (int i = 0; i < 4; i++) {
            int nx = u.x + dx[i];
            int nz = u.z + dz[i];
            Node target = new Node(nx, u.y + 1, nz);
            
            // Can only jump if head space is clear
            if (world.isPassable(new Vector3d(u.x, u.y + 2, u.z))) {
                if (isStandable(nx, u.y + 1, nz, world)) {
                    edges.add(new Edge(target, getEuclideanDistance(u, target) + 0.5));
                } else if (canDigThrough(nx, u.y + 1, nz, world)) {
                    edges.add(new Edge(target, getEuclideanDistance(u, target) + 100.5));
                }
            }
        }
        return edges;
    }
}
