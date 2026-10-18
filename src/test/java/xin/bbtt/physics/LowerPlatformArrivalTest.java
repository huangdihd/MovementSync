package xin.bbtt.physics;

import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class LowerPlatformArrivalTest {
    @Test void directLowerPlatformCannotBeNormalRouteTouchdownEvenWithBroadArrivalRadius() {
        BlockState stone = new BlockState("minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                // Loaded lower ground directly underneath the requested elevated platform.
                return p.y < 40 || p.y >= 59 && p.y < 60 && p.x >= 175 && p.x < 177
                        ? stone : super.getBlockStateAt(p);
            }
        };
        Vector3d goal = new Vector3d(175.5, 60, -31.5);
        PhysicsState initial = new PhysicsState(new Vector3d(175.5, 40.01, -31.5),
                new Vector3d(0, -.1, 0), false, 0, PlayerPose.GLIDING);
        var route = new WaypointFlightTarget(List.of(initial.position(), goal), goal, true, false, 64);
        // Every unpowered candidate touches lower support immediately. A radius of
        // 32 must not turn the twenty-block height error into a normal arrival.
        assertThrows(IllegalStateException.class, () -> new ElytraFlightController().decide(
                world, initial, 0, 0, false, route, 0, 32));
        // Independent emergency landing remains allowed to select this safe ground.
        Vector3d safe = ElytraFlightController.landingSite(world, 175.5, -31.5, 40.01);
        assertNotNull(safe);
        assertEquals(40, safe.y, .001);
    }
}
