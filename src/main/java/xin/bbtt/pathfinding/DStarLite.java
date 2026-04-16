package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import xin.bbtt.world.World;
import xin.bbtt.mcbot.LangManager;

import java.util.*;

public class DStarLite {
    private final Node start;
    private final Node goal;
    private final World world;

    public DStarLite(Node start, Node goal, World world) {
        this.start = start;
        this.goal = goal;
        this.world = world;
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

    private double heuristic(Node a, Node b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2) + Math.pow(a.z - b.z, 2));
    }

    private List<Node> getSuccessors(Node u) {
        List<Node> neighbors = new ArrayList<>();
        int[] dx = {1, -1, 0, 0, 1, 1, -1, -1};
        int[] dz = {0, 0, 1, -1, 1, -1, 1, -1};

        boolean hasBlocks = xin.bbtt.MovementSync.Instance.getInventoryManager().findBlock() != -1;

        for (int i = 0; i < 8; i++) {
            int nx = u.x + dx[i];
            int nz = u.z + dz[i];
            
            // 1. Walk / Dig through obstacles
            if (isStandable(nx, u.y, nz)) {
                neighbors.add(new Node(nx, u.y, nz));
            } else if (i < 4 && canDigThrough(nx, u.y, nz)) {
                neighbors.add(new Node(nx, u.y, nz));
            }

            // Cardinal only movements for complexity reduction
            if (i < 4) {
                // 2. Jump Up (1 block)
                if (isPassable(nx, u.y + 2, nz)) {
                    if (isStandable(nx, u.y + 1, nz)) {
                        neighbors.add(new Node(nx, u.y + 1, nz));
                    } else if (canDigThrough(nx, u.y + 1, nz)) {
                        neighbors.add(new Node(nx, u.y + 1, nz));
                    }
                }

                // 3. Fall Down (up to 3 blocks)
                for (int dy = -1; dy >= -3; dy--) {
                    if (isStandable(nx, u.y + dy, nz)) {
                        neighbors.add(new Node(nx, u.y + dy, nz));
                        break;
                    }
                    if (!isPassable(nx, u.y + dy, nz)) break; 
                }

                // 4. Jump Gaps (1-2 blocks)
                if (isPassable(u.x, u.y + 2, u.z)) {
                    for (int gap = 1; gap <= 2; gap++) {
                        int tx = u.x + (gap + 1) * dx[i];
                        int tz = u.z + (gap + 1) * dz[i];
                        
                        boolean gapPassable = true;
                        for (int step = 1; step <= gap; step++) {
                            int midX = u.x + step * dx[i];
                            int midZ = u.z + step * dz[i];
                            if (!isPassable(midX, u.y, midZ) || !isPassable(midX, u.y + 1, midZ) || !isPassable(midX, u.y + 2, midZ)) {
                                gapPassable = false;
                                break;
                            }
                        }
                        
                        if (gapPassable && isStandable(tx, u.y, tz) && isPassable(tx, u.y + 2, tz)) {
                            neighbors.add(new Node(tx, u.y, tz));
                        }
                    }
                }

                // 5. Bridge / Pillar Up (Jump & Place) - Only if we have blocks
                if (hasBlocks && isPassable(u.x, u.y + 2, u.z)) {
                    // Pillar Up
                    neighbors.add(new Node(u.x, u.y + 1, u.z));
                    
                    // Bridge (if not already handled by jump gaps)
                    int bx = u.x + dx[i];
                    int bz = u.z + dz[i];
                    if (isPassable(bx, u.y, bz) && isPassable(bx, u.y + 1, bz)) {
                        neighbors.add(new Node(bx, u.y, bz));
                    }
                }
            }
        }
        return neighbors;
    }

    private boolean canDigThrough(int x, int y, int z) {
        if (!world.chunkLoaded(x >> 4, z >> 4)) return false;
        if (!world.getBlockStateAt(new Vector3d(x, y - 1, z)).isSolid()) return false;
        xin.bbtt.Block.BlockState feet = world.getBlockStateAt(new Vector3d(x, y, z));
        xin.bbtt.Block.BlockState head = world.getBlockStateAt(new Vector3d(x, y + 1, z));
        if (feet.isPassable() && head.isPassable()) return false; 
        return (feet.diggable() || feet.isPassable()) && (head.diggable() || head.isPassable());
    }

    private boolean isPassable(int x, int y, int z) {
        return world.isPassable(new Vector3d(x, y, z));
    }

    private boolean isStandable(int x, int y, int z) {
        if (!world.chunkLoaded(x >> 4, z >> 4)) return false;
        return isPassable(x, y, z) && isPassable(x, y + 1, z) && world.getBlockStateAt(new Vector3d(x, y - 1, z)).isSolid();
    }

    private double cost(Node a, Node b) {
        double baseCost = heuristic(a, b);
        
        if (!isPassable(b.x, b.y, b.z) || !isPassable(b.x, b.y + 1, b.z)) {
            baseCost += 100.0; 
        }

        if (!world.getBlockStateAt(new Vector3d(b.x, b.y - 1, b.z)).isSolid()) {
            baseCost += 5.0; 
        }

        double horizontalDistSq = Math.pow(a.x - b.x, 2) + Math.pow(a.z - b.z, 2);
        if (horizontalDistSq > 1.5) {
            baseCost += 0.5; 
        }

        if (b.y < a.y) baseCost += (a.y - b.y) * 2.0; 
        if (b.y > a.y) baseCost += 0.5;
        return baseCost;
    }

    public List<Node> findPath(int maxIterations) {
        long startTime = System.currentTimeMillis();
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
                List<Node> path = reconstructPath(cameFrom, curr);
                xin.bbtt.MovementSync.Instance.getLogger().info(LangManager.get("movementsync.command.goto.success", path.size(), iterations, System.currentTimeMillis() - startTime));
                return path;
            }
            double h = heuristic(curr, goal);
            if (h < minH) { minH = h; bestNode = curr; }
            for (Node neighbor : getSuccessors(curr)) {
                double tentativeG = gScore.getOrDefault(curr, Double.POSITIVE_INFINITY) + cost(curr, neighbor);
                if (tentativeG < gScore.getOrDefault(neighbor, Double.POSITIVE_INFINITY)) {
                    cameFrom.put(neighbor, curr);
                    gScore.put(neighbor, tentativeG);
                    openSet.removeIf(e -> e.node.equals(neighbor));
                    openSet.add(new NodeEntry(neighbor, tentativeG + heuristic(neighbor, goal), tentativeG));
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
