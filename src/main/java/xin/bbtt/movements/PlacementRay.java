package xin.bbtt.movements;

import org.cloudburstmc.math.vector.Vector3i;
import org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction;
import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.collision.CollisionShapeRegistry;
import xin.bbtt.world.World;

/** Placement-only ray against loaded block shapes; never guesses a cursor. */
final class PlacementRay {
    record Hit(Vector3i block, Direction face, Vector3d cursor) {}

    static Hit cast(World world, MovementSync.PlacementFrame frame) {
        double yaw = Math.toRadians(frame.yaw()), pitch = Math.toRadians(frame.pitch());
        double[] origin = {frame.x(), frame.y() + frame.eyeHeight(), frame.z()};
        double[] direction = {-Math.sin(yaw) * Math.cos(pitch), -Math.sin(pitch),
                Math.cos(yaw) * Math.cos(pitch)};
        double nearest = 4.5;
        Hit hit = null;
        Direction[] negative = {Direction.WEST, Direction.DOWN, Direction.NORTH};
        Direction[] positive = {Direction.EAST, Direction.UP, Direction.SOUTH};
        for (int x = (int) Math.floor(origin[0] - 4.5); x <= Math.floor(origin[0] + 4.5); x++) {
            for (int z = (int) Math.floor(origin[2] - 4.5); z <= Math.floor(origin[2] + 4.5); z++) {
                if (!world.chunkLoaded(x >> 4, z >> 4)) continue;
                for (int y = (int) Math.floor(origin[1] - 4.5); y <= Math.floor(origin[1] + 4.5); y++) {
                    var state = world.getBlockStateAt(new Vector3d(x, y, z));
                    for (var box : CollisionShapeRegistry.getInstance().boxesFor(state.stateId())) {
                        double[] min = {x + box.minX(), y + box.minY(), z + box.minZ()};
                        double[] max = {x + box.maxX(), y + box.maxY(), z + box.maxZ()};
                        double enter = 0, exit = nearest;
                        Direction face = null;
                        for (int axis = 0; axis < 3; axis++) {
                            if (Math.abs(direction[axis]) < 1.0e-10) {
                                if (origin[axis] < min[axis] || origin[axis] > max[axis]) exit = -1;
                                continue;
                            }
                            double a = (min[axis] - origin[axis]) / direction[axis];
                            double b = (max[axis] - origin[axis]) / direction[axis];
                            double near = Math.min(a, b);
                            if (near > enter) {
                                enter = near;
                                face = direction[axis] > 0 ? negative[axis] : positive[axis];
                            }
                            exit = Math.min(exit, Math.max(a, b));
                        }
                        if (face != null && enter <= exit && enter < nearest) {
                            nearest = enter;
                            hit = new Hit(Vector3i.from(x, y, z), face,
                                    new Vector3d(origin[0] + direction[0] * enter - x,
                                            origin[1] + direction[1] * enter - y,
                                            origin[2] + direction[2] * enter - z));
                        }
                    }
                }
            }
        }
        return hit;
    }

    static Vector3d aim(Vector3i block, Direction face) {
        Vector3d point = new Vector3d(block.getX() + 0.5, block.getY() + 0.5, block.getZ() + 0.5);
        switch (face) {
            case DOWN -> point.y -= 0.5;
            case UP -> point.y += 0.5;
            case NORTH -> point.z -= 0.5;
            case SOUTH -> point.z += 0.5;
            case WEST -> point.x -= 0.5;
            case EAST -> point.x += 0.5;
        }
        return point;
    }
}
