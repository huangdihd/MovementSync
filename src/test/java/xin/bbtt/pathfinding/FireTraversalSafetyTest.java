package xin.bbtt.pathfinding;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;
import java.util.Map;
import static org.junit.jupiter.api.Assertions.*;

class FireTraversalSafetyTest {
    private World fire(String name) {
        var air = new BlockState("air", 0, Map.of(), "empty", 0, false, "AIR");
        var hazard = new BlockState(name, 0, Map.of(), "empty", 0, false, "FIRE");
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return Math.floor(p.x) == 27 && Math.floor(p.y) == 38
                        && Math.floor(p.z) == -8 ? hazard : air;
            }
        };
    }

    @Test void emptyShapeFireAtRecordedStopTouchesBodyOutsideFeetCell() {
        for (String name : new String[]{"fire", "soul_fire", "minecraft:fire"}) {
            World world = fire(name);
            assertFalse(TraversalSafety.isSafe(world, new Vector3d(26.8, 38, -7.61)), name);
            assertTrue(TraversalSafety.isSafe(world, new Vector3d(26.69, 38, -7.61)), name);
            assertFalse(TraversalSafety.isSafe(world, new Vector3d(26.8, 37, -7.61)),
                    "upper body also contacts fire");
        }
    }

    @Test void routeCannotCrossFireEvenWhenEndpointsAreClear() {
        World world = fire("soul_fire");
        var from = new Vector3d(26, 38, -7.5);
        var to = new Vector3d(29, 38, -7.5);
        assertTrue(TraversalSafety.isSafe(world, from));
        assertTrue(TraversalSafety.isSafe(world, to));
        assertFalse(TraversalSafety.isSafeSegment(world, from, to));
    }
}
