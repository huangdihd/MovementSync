package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import xin.bbtt.world.World;

import java.util.*;

public class DStarLite {
    private final Node start;
    private final Node goal;
    private final World world;

    private static class NodeEntry implements Comparable<NodeEntry> {
        Node node;
        double f; // f = g + h
        double g;

        NodeEntry(Node node, double f, double g) {
            this.node = node;
            this.f = f;
            this.g = g;
        }

        @Override
        public int compareTo(NodeEntry o) {
            return Double.compare(this.f, o.f);
        }
    }

    public DStarLite(Node start, Node goal, World world) {
        this.start = start;
        this.goal = goal;
        this.world = world;
    }

    private double heuristic(Node a, Node b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2) + Math.pow(a.z - b.z, 2));
    }

    private List<Node> getSuccessors(Node u) {
        List<Node> neighbors = new ArrayList<>();
        // 8-way movement: 4 cardinal + 4 diagonal
        int[] dx = {1, -1, 0, 0, 1, 1, -1, -1};
        int[] dz = {0, 0, 1, -1, 1, -1, 1, -1};

        for (int i = 0; i < 8; i++) {
            int nx = u.x + dx[i];
            int nz = u.z + dz[i];
            
            // Flat walk
            if (isStandable(nx, u.y, nz)) {
                neighbors.add(new Node(nx, u.y, nz));
            } else {
                // Only allow jumping/falling for cardinal directions to keep it simple and safe
                if (i < 4) {
                    // Jump up (1 block high)
                    if (isPassable(nx, u.y + 2, nz) && isStandable(nx, u.y + 1, nz)) {
                        neighbors.add(new Node(nx, u.y + 1, nz));
                    }
                    // Jump over gaps (1 to 2 block gaps)
                    if (isPassable(u.x, u.y + 2, u.z)) {
                        for (int gap = 1; gap <= 2; gap++) {
                            int targetX = u.x + (gap + 1) * dx[i];
                            int targetZ = u.z + (gap + 1) * dz[i];
                            
                            // Check all blocks in the gap are passable
                            boolean gapPassable = true;
                            for (int step = 1; step <= gap; step++) {
                                int midX = u.x + step * dx[i];
                                int midZ = u.z + step * dz[i];
                                if (!isPassable(midX, u.y, midZ) || !isPassable(midX, u.y + 1, midZ) || !isPassable(midX, u.y + 2, midZ)) {
                                    gapPassable = false;
                                    break;
                                }
                            }
                            
                            if (gapPassable && isStandable(targetX, u.y, targetZ) && isPassable(targetX, u.y + 2, targetZ)) {
                                neighbors.add(new Node(targetX, u.y, targetZ));
                            }
                        }
                    }
                    // Fall down (up to 3 blocks)
                    for (int dy = -1; dy >= -3; dy--) {
                        if (isStandable(nx, u.y + dy, nz)) {
                            neighbors.add(new Node(nx, u.y + dy, nz));
                            break;
                        }
                        if (!isPassable(nx, u.y + dy, nz)) break; 
                    }
                }
            }
        }
        return neighbors;
    }

    private boolean isPassable(int x, int y, int z) {
        return world.isPassable(new Vector3d(x, y, z));
    }

    private boolean isStandable(int x, int y, int z) {
        // Check if chunk is loaded. If not, we can't safely navigate through it.
        if (!world.chunkLoaded(x >> 4, z >> 4)) return false;

        // To stand at (x, y, z): 
        // (x, y, z) must be passable (feet space)
        // (x, y+1, z) must be passable (head space)
        // (x, y-1, z) must be SOLID (ground)
        return isPassable(x, y, z) && isPassable(x, y + 1, z) && world.getBlockStateAt(new Vector3d(x, y - 1, z)).isSolid();
    }

    private double cost(Node a, Node b) {
        double baseCost = heuristic(a, b);
        
        // Penalize falling to encourage staying on the same level
        if (b.y < a.y) {
            baseCost += (a.y - b.y) * 2.0; 
        }
        
        // Penalize jumping slightly
        if (b.y > a.y) {
            baseCost += 0.5;
        }

        return baseCost;
    }

    public List<Node> findPath(int maxIterations) {
        PriorityQueue<NodeEntry> openSet = new PriorityQueue<>();
        Map<Node, Double> gScore = new HashMap<>();
        Map<Node, Node> cameFrom = new HashMap<>();
        
        Node bestNode = start;
        double minH = heuristic(start, goal);

        openSet.add(new NodeEntry(start, minH, 0.0));
        gScore.put(start, 0.0);

        int iterations = 0;
        while (!openSet.isEmpty()) {
            if (iterations++ > maxIterations) break;

            Node curr = openSet.poll().node;

            if (curr.equals(goal)) {
                return reconstructPath(cameFrom, curr);
            }
            
            // Track the node that gets us closest to the target
            double h = heuristic(curr, goal);
            if (h < minH) {
                minH = h;
                bestNode = curr;
            }

            for (Node neighbor : getSuccessors(curr)) {
                double tentativeG = gScore.getOrDefault(curr, Double.POSITIVE_INFINITY) + cost(curr, neighbor);

                if (tentativeG < gScore.getOrDefault(neighbor, Double.POSITIVE_INFINITY)) {
                    cameFrom.put(neighbor, curr);
                    gScore.put(neighbor, tentativeG);
                    double fScore = tentativeG + heuristic(neighbor, goal);
                    
                    openSet.removeIf(e -> e.node.equals(neighbor));
                    openSet.add(new NodeEntry(neighbor, fScore, tentativeG));
                }
            }
        }

        // Partial path support: if goal not reachable, return path to the closest known node
        if (!bestNode.equals(start)) {
            return reconstructPath(cameFrom, bestNode);
        }

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
