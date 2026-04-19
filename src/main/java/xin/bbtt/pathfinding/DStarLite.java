package xin.bbtt.pathfinding;

import xin.bbtt.world.World;
import xin.bbtt.mcbot.LangManager;

import java.util.*;

public class DStarLite {
    private final Node start;
    private final Node goal;
    private final PathfindingContext context;

    public DStarLite(Node start, Node goal, World world) {
        this(start, goal, new DefaultPathfindingContext(world));
    }

    public DStarLite(Node start, Node goal, PathfindingContext context) {
        this.start = start;
        this.goal = goal;
        this.context = context;
    }

    private static class NodeEntry implements Comparable<NodeEntry> {
        Node node;
        double f;
        double g;

        NodeEntry(Node node, double f, double g) {
            this.node = node;
            this.f = f;
            this.g = g;
        }

        @Override
        public int compareTo(NodeEntry o) {
            int cmp = Double.compare(this.f, o.f);
            if (cmp == 0) return Double.compare(this.g, o.g);
            return cmp;
        }
    }

    public List<Node> findPath(int maxIterations) {
        long startTime = System.currentTimeMillis();
        PriorityQueue<NodeEntry> openSet = new PriorityQueue<>();
        Map<Node, Double> gScore = new HashMap<>();
        Map<Node, Node> cameFrom = new HashMap<>();
        
        Node bestNode = start;
        double minH = context.getHeuristic(start, goal);
        
        openSet.add(new NodeEntry(start, minH, 0.0));
        gScore.put(start, 0.0);
        
        int iterations = 0;
        while (!openSet.isEmpty()) {
            if (iterations++ > maxIterations) break;
            Node curr = openSet.poll().node;
            
            if (curr.equals(goal)) {
                List<Node> path = reconstructPath(cameFrom, curr);
                xin.bbtt.MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.goto.success", path.size(), iterations, System.currentTimeMillis() - startTime));
                return path;
            }
            
            double h = context.getHeuristic(curr, goal);
            if (h < minH) { 
                minH = h; 
                bestNode = curr; 
            }
            
            for (Edge edge : context.getEdges(curr)) {
                Node neighbor = edge.getTarget();
                double tentativeG = gScore.getOrDefault(curr, Double.POSITIVE_INFINITY) + edge.getCost();
                if (tentativeG < gScore.getOrDefault(neighbor, Double.POSITIVE_INFINITY)) {
                    cameFrom.put(neighbor, curr);
                    gScore.put(neighbor, tentativeG);
                    openSet.removeIf(e -> e.node.equals(neighbor));
                    openSet.add(new NodeEntry(neighbor, tentativeG + context.getHeuristic(neighbor, goal), tentativeG));
                }
            }
        }
        
        xin.bbtt.MovementSync.Instance.getLogger().warn(LangManager.get("movementsync.command.goto.not_found", goal.x, goal.y, goal.z, iterations));
        if (!bestNode.equals(start)) return reconstructPath(cameFrom, bestNode);
        return new ArrayList<>();
    }

    private List<Node> reconstructPath(Map<Node, Node> cameFrom, Node current) {
        List<Node> path = new ArrayList<>();
        path.add(current);
        while (cameFrom.containsKey(current)) {
            current = cameFrom.get(current);
            path.add(current);
        }
        Collections.reverse(path);
        return path;
    }
}
