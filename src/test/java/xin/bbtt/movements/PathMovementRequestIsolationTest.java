package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.world.World;

final class PathMovementRequestIsolationTest {
    private static final BlockState STONE = new BlockState(
        "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
        "minecraft:air", 0, Map.of(), "empty", 0.0, false, "AIR");

    @Test
    void internalRepathKeepsTheGoalCapturedByItsOwnRequest() throws Exception {
        MovementSync movementSync = new MovementSync();
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, flatWorld());
        movementSync.position.set(new Vector3d(0.5, 64.0, 0.5));
        movementSync.setFollowTargetId(-1);
        Vector3i originalGoal = new Vector3i(2, 64, 0);
        movementSync.setActiveGoal(originalGoal);
        PathMovement oldDigAllowedRequest = new PathMovement(
            List.of(
                new PathStep(new Node(0, 64, 0), BuiltinMovementType.WALK),
                new PathStep(new Node(2, 64, 0), BuiltinMovementType.WALK)
            ),
            true
        );

        movementSync.setActiveGoal(new Vector3i(0, 64, 2));
        Method repath = PathMovement.class.getDeclaredMethod("repathInternally");
        repath.setAccessible(true);
        repath.invoke(oldDigAllowedRequest);

        Field pathField = PathMovement.class.getDeclaredField("path");
        pathField.setAccessible(true);
        @SuppressWarnings("unchecked")
        List<PathStep> replanned = (List<PathStep>) pathField.get(oldDigAllowedRequest);
        assertEquals(new Node(originalGoal.x, originalGoal.y, originalGoal.z),
            replanned.get(replanned.size() - 1).getNode(),
            "a prior request must not adopt a later request's global target");
    }

    @Test
    void staticPathIsSupersededWhenAFollowRequestStarts() throws Exception {
        MovementSync movementSync = new MovementSync();
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, flatWorld());
        movementSync.position.set(new Vector3d(0.5, 64.0, 0.5));
        movementSync.setFollowTargetId(-1);
        movementSync.setActiveGoal(new Vector3i(2, 64, 0));
        PathMovement staticPath = new PathMovement(
            List.of(
                new PathStep(new Node(0, 64, 0), BuiltinMovementType.WALK),
                new PathStep(new Node(2, 64, 0), BuiltinMovementType.WALK)
            ),
            true
        );

        movementSync.setFollowTargetId(42);
        staticPath.onTick();

        assertTrue(staticPath.isFinished(),
            "a later follow request must supersede an older static path");
    }

    @Test
    void olderPathCannotClearANewerRequestAtTheSameCoordinates() throws Exception {
        MovementSync movementSync = new MovementSync();
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, flatWorld());
        movementSync.position.set(new Vector3d(0.5, 64.0, 0.5));
        Vector3i sameGoal = new Vector3i(2, 64, 0);
        movementSync.beginNavigationRequest();
        movementSync.setFollowTargetId(-1);
        movementSync.setActiveGoal(sameGoal);
        PathMovement older = new PathMovement(
            List.of(
                new PathStep(new Node(0, 64, 0), BuiltinMovementType.WALK),
                new PathStep(new Node(2, 64, 0), BuiltinMovementType.WALK)
            ),
            true
        );

        movementSync.beginNavigationRequest();
        movementSync.setActiveGoal(new Vector3i(sameGoal));
        older.onTick();

        assertTrue(older.isFinished());
        assertEquals(sameGoal, movementSync.getActiveGoal(),
            "an older generation must not clear a newer same-coordinate request");
    }

    private static World flatWorld() {
        return new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public BlockState getBlockStateAt(Vector3d position) {
                return Math.floor(position.y) == 63 ? STONE : AIR;
            }
        };
    }
}
