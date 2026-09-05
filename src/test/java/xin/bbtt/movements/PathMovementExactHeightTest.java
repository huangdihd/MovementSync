package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.util.List;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.pathfinding.StandablePositionResolver;
import xin.bbtt.world.World;

final class PathMovementExactHeightTest {
    private static final int AIR = 0;
    private static final int BOTTOM_OAK_SLAB = 13131;

    @AfterEach
    void resetSingleton() {
        MovementSync.INSTANCE = null;
    }

    @Test
    void bottomSlabNodeUsesExactHalfBlockFeetHeight() throws Exception {
        MovementSync movementSync = new MovementSync();
        World world = slabWorld();
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.position.set(new Vector3d(0.5, 0.5, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        Vector3i goal = new Vector3i(0, 1, 0);
        long generation = movementSync.beginStaticNavigationRequest(goal);
        PathMovement movement = new PathMovement(
            List.of(new PathStep(new Node(0, 1, 0), BuiltinMovementType.WALK)),
            -1,
            false,
            goal,
            -1,
            generation
        );

        assertEquals(0.5,
            StandablePositionResolver.feetY(world, 0, 1, 0).orElseThrow(), 1.0e-9);
        assertEquals(1, StandablePositionResolver.nodeY(0.5));
        assertEquals(1, StandablePositionResolver.nodeY(1.0));
        assertEquals(0, StandablePositionResolver.nodeY(-0.5));
        movement.onTick();

        assertTrue(movement.isFinished(),
            "standing at the exact slab top must complete integer planner node y=1");
    }

    private static World slabWorld() {
        return new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                return x == 0 && y == 0 && z == 0 ? BOTTOM_OAK_SLAB : AIR;
            }
        };
    }
}
