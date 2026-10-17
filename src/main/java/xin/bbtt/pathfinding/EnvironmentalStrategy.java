package xin.bbtt.pathfinding;

import java.util.ArrayList;
import java.util.List;
import xin.bbtt.world.World;

/** Surface swimming and climbable columns, including entry and exit to solid ground. */
public final class EnvironmentalStrategy implements MovementStrategy {
    @Override
    public List<Edge> findEdges(Node source, World world) {
        List<Edge> edges = new ArrayList<>();
        var start = EnvironmentalTraversal.position(world, source);
        if (start.isEmpty() && EnvironmentalTraversal.recoveryPosition(world, source).isPresent()) {
            Node surfaceward = new Node(source.x, source.y + 1, source.z);
            if (EnvironmentalTraversal.canTraverse(world, source, surfaceward)) {
                edges.add(new Edge(surfaceward, 2, BuiltinMovementType.SWIM));
            }
            return edges;
        }
        for (int deltaX = -1; deltaX <= 1; deltaX++) {
            for (int deltaZ = -1; deltaZ <= 1; deltaZ++) {
                if (Math.abs(deltaX) + Math.abs(deltaZ) > 1) continue;
                for (int deltaY = -1; deltaY <= 1; deltaY++) {
                    if (deltaX == 0 && deltaZ == 0 && deltaY == 0) continue;
                    Node target = new Node(source.x + deltaX, source.y + deltaY, source.z + deltaZ);
                    var end = EnvironmentalTraversal.position(world, target);
                    if (start.isEmpty() && end.isEmpty()) continue;
                    // Recovery nodes are only entered from below, never as a
                    // shortcut that intentionally dives from a surface route.
                    if (end.isEmpty() && EnvironmentalTraversal.recoveryPosition(world, target).isPresent()) continue;
                    boolean climb = start.map(position -> EnvironmentalTraversal.climbing(world, position)).orElse(false)
                            || end.map(position -> EnvironmentalTraversal.climbing(world, position)).orElse(false);
                    if (deltaX == 0 && deltaZ == 0 && !climb) continue;
                    if (!EnvironmentalTraversal.canTraverse(world, source, target)) continue;
                    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
                    // Surface swimming is well under half of walking speed; weight it so
                    // routes over land are preferred whenever a reasonable detour exists.
                    edges.add(new Edge(target, distance * 3,
                            climb ? BuiltinMovementType.CLIMB : BuiltinMovementType.SWIM));
                }
            }
        }
        return edges;
    }
}
