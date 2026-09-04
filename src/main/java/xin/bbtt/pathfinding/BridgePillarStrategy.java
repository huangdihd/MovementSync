package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import xin.bbtt.world.World;
import xin.bbtt.MovementSync;

import java.util.ArrayList;
import java.util.List;

/**
 * Strategy for bridging gaps and pillaring up using blocks in inventory.
 */
public class BridgePillarStrategy extends AbstractMovementStrategy {
    @Override
    public List<Edge> findEdges(Node u, World world) {
        List<Edge> edges = new ArrayList<>();
        boolean hasBlocks = MovementSync.INSTANCE.getInventoryManager().findBlock() != -1;
        if (!hasBlocks) return edges;
        
        // Needs head space to jump/pillar
        if (!world.isPassable(new Vector3d(u.x, u.y + 2, u.z))) return edges;

        // Pillar Up
        Node pillarTarget = new Node(u.x, u.y + 1, u.z);
        if (!world.getBlockStateAt(new Vector3d(pillarTarget.x, pillarTarget.y - 1, pillarTarget.z)).isSolid()
                && StandablePositionResolver.canJumpToFeetY(world, u, pillarTarget.y)) {
            edges.add(new Edge(pillarTarget, getEuclideanDistance(u, pillarTarget) + 5.0, BuiltinMovementType.PILLAR));
        }
        
        // Bridge
        int[] dx = {1, -1, 0, 0};
        int[] dz = {0, 0, 1, -1};
        for (int i = 0; i < 4; i++) {
            int bx = u.x + dx[i];
            int bz = u.z + dz[i];
            if (world.isPassable(new Vector3d(bx, u.y, bz)) && world.isPassable(new Vector3d(bx, u.y + 1, bz))) {
                if (!world.getBlockStateAt(new Vector3d(bx, u.y - 1, bz)).isSolid()
                        && StandablePositionResolver.canStepToFeetY(world, u, u.y)) {
                    Node bridgeTarget = new Node(bx, u.y, bz);
                    edges.add(new Edge(bridgeTarget, getEuclideanDistance(u, bridgeTarget) + 5.0, BuiltinMovementType.BRIDGE));
                }
            }
        }
        return edges;
    }
}
