package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import xin.bbtt.world.World;

public abstract class AbstractMovementStrategy implements MovementStrategy {
    
    protected boolean isStandable(int x, int y, int z, World world) {
        return StandablePositionResolver.feetY(world, x, y, z).isPresent();
    }

    protected boolean canDigThrough(int x, int y, int z, World world) {
        if (!world.chunkLoaded(x >> 4, z >> 4)) return false;
        if (StandablePositionResolver.supportY(world, x, y, z).isEmpty()) return false;
        xin.bbtt.Block.BlockState feet = world.getBlockStateAt(new Vector3d(x, y, z));
        xin.bbtt.Block.BlockState head = world.getBlockStateAt(new Vector3d(x, y + 1, z));
        if (feet.isPassable() && head.isPassable()) return false; 
        return (feet.diggable() || feet.isPassable()) && (head.diggable() || head.isPassable());
    }

    protected double getEuclideanDistance(Node a, Node b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2) + Math.pow(a.z - b.z, 2));
    }
}
