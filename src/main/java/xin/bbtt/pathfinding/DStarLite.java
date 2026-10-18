package xin.bbtt.pathfinding;

import xin.bbtt.world.World;
import xin.bbtt.mcbot.LangManager;

import java.util.*;

public class DStarLite {
    public enum Termination { COMPLETE, BUDGET_EXHAUSTED, UNLOADED_FRONTIER, EXHAUSTED, INVALID_BUDGET }

    /** Expanded nodes count valid queue removals, including a terminal goal/frontier. */
    public record SearchDiagnostics(Termination termination, int expandedNodes, int budget,
                                    int openNodes, Node endpoint, int pathNodes) {}

    private SearchDiagnostics diagnostics;

    /** Result of the most recent search, or null before findPath is called. */
    public SearchDiagnostics diagnostics() { return diagnostics; }

    private List<PathStep> result(List<PathStep> path, Termination termination,
                                  int expanded, int budget, int openNodes) {
        diagnostics = new SearchDiagnostics(termination, expanded, budget, openNodes,
                path.isEmpty() ? null : path.get(path.size() - 1).getNode(), path.size());
        xin.bbtt.MovementSync.getLogger().debug(
                "Path search start={} goal={} termination={} expanded={} budget={} open={} endpoint={} pathNodes={}",
                start, goal, termination, expanded, budget, openNodes, diagnostics.endpoint(), path.size());
        return path;
    }

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

    public List<PathStep> findPath(int maxIterations) {
        if (maxIterations < 0) return result(new ArrayList<>(), Termination.INVALID_BUDGET, 0, maxIterations, 0);
        if (start.equals(goal)) return result(new ArrayList<>(List.of(new PathStep(start, BuiltinMovementType.WALK))),
                Termination.COMPLETE, 0, maxIterations, 0);
        try (StandablePositionResolver.SearchCache ignored = StandablePositionResolver.beginSearch()) {
            return search(maxIterations);
        }
    }

    private List<PathStep> search(int maxIterations) {
        long startTime = System.currentTimeMillis();
        PriorityQueue<NodeEntry> openSet = new PriorityQueue<>();
        Map<Node, NodeEntry> active = new HashMap<>();
        Map<Node, Double> gScore = new HashMap<>();
        Map<Node, Node> cameFromNode = new HashMap<>();
        Map<Node, MovementType> cameFromType = new HashMap<>();
        
        Node bestNode = start;
        double minH = context.getHeuristic(start, goal);
        
        NodeEntry initial = new NodeEntry(start, minH, 0.0);
        openSet.add(initial);
        active.put(start, initial);
        gScore.put(start, 0.0);
        
        int iterations = 0;
        while (!active.isEmpty()) {
            NodeEntry entry = openSet.poll();
            if (active.get(entry.node) != entry) continue;
            Node curr = entry.node;
            
            if (curr.equals(goal)) {
                List<PathStep> path = reconstructPath(cameFromNode, cameFromType, curr);
                xin.bbtt.MovementSync.getLogger().info(LangManager.get("movementsync.command.goto.success", path.size(), iterations, System.currentTimeMillis() - startTime));
                return result(path, Termination.COMPLETE, iterations, maxIterations, active.size());
            }
            
            // Recognizing the goal is not an expansion (no outgoing edges).
            if (iterations >= maxIterations) break;
            active.remove(curr);
            iterations++;
            double h = context.getHeuristic(curr, goal);
            if (h < minH) { 
                minH = h; 
                bestNode = curr; 
            }

            // A partial route is useful immediately once the closest search
            // frontier reaches unknown terrain toward an unloaded goal.
            if (h <= minH && context.isUnloadedFrontier(curr, goal)
                    && active.values().stream().noneMatch(candidate -> context.getHeuristic(candidate.node, goal) < h)) {
                return result(reconstructPath(cameFromNode, cameFromType, curr),
                        Termination.UNLOADED_FRONTIER, iterations, maxIterations, active.size());
            }
            
            for (Edge edge : context.getEdges(curr)) {
                Node neighbor = edge.getTarget();
                double tentativeG = gScore.getOrDefault(curr, Double.POSITIVE_INFINITY) + edge.getCost();
                if (tentativeG < gScore.getOrDefault(neighbor, Double.POSITIVE_INFINITY)) {
                    cameFromNode.put(neighbor, curr);
                    cameFromType.put(neighbor, edge.getType());
                    gScore.put(neighbor, tentativeG);
                    NodeEntry replacement = new NodeEntry(neighbor,
                            tentativeG + context.getHeuristic(neighbor, goal), tentativeG);
                    active.put(neighbor, replacement);
                    openSet.add(replacement);
                }
            }
        }
        
        xin.bbtt.MovementSync.getLogger().debug(LangManager.get("movementsync.command.goto.not_found", goal.x, goal.y, goal.z, iterations));
        List<PathStep> partial = bestNode.equals(start) ? new ArrayList<>()
                : reconstructPath(cameFromNode, cameFromType, bestNode);
        return result(partial, active.isEmpty() ? Termination.EXHAUSTED : Termination.BUDGET_EXHAUSTED,
                iterations, maxIterations, active.size());
    }

    private List<PathStep> reconstructPath(Map<Node, Node> cameFromNode, Map<Node, MovementType> cameFromType, Node current) {
        List<PathStep> path = new ArrayList<>();
        while (cameFromNode.containsKey(current)) {
            MovementType type = cameFromType.get(current);
            path.add(new PathStep(current, type));
            current = cameFromNode.get(current);
        }
        path.add(new PathStep(current, BuiltinMovementType.WALK));
        Collections.reverse(path);
        return path;
    }
}
