package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import xin.bbtt.world.World;
import java.util.ArrayList;
import java.util.List;

/**
 * Strategy for jumping over gaps (1-2 blocks wide).
 */
public class GapJumpStrategy extends AbstractMovementStrategy {
    @Override
    public List<Edge> findEdges(Node u, World world) {
        List<Edge> edges = new ArrayList<>();
        int[] dx = {1, -1, 0, 0};
        int[] dz = {0, 0, 1, -1};

        // Needs head space to jump
        if (!world.isPassable(new Vector3d(u.x, u.y + 2, u.z))) return edges;

        for (int i = 0; i < 4; i++) {
            for (int gap = 1; gap <= 2; gap++) {
                int tx = u.x + (gap + 1) * dx[i];
                int tz = u.z + (gap + 1) * dz[i];

                boolean gapPassable = true;
                for (int step = 1; step <= gap; step++) {
                    int midX = u.x + step * dx[i];
                    int midZ = u.z + step * dz[i];
                    if (!world.isPassable(new Vector3d(midX, u.y, midZ)) || 
                        !world.isPassable(new Vector3d(midX, u.y + 1, midZ)) || 
                        !world.isPassable(new Vector3d(midX, u.y + 2, midZ))) {
                        gapPassable = false;
                        break;
                    }
                }

                if (gapPassable && isStandable(tx, u.y, tz, world) && world.isPassable(new Vector3d(tx, u.y + 2, tz))) {
                    Node target = new Node(tx, u.y, tz);
                    edges.add(new Edge(target, getEuclideanDistance(u, target) + 1.0));
                }
            }
        }
        return edges;
    }
}
