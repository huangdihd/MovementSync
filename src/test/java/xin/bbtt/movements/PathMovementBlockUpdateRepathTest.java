package xin.bbtt.movements;

import org.junit.jupiter.api.Test;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;

import java.lang.reflect.Field;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PathMovementBlockUpdateRepathTest {
    private static PathMovement straightPath() {
        return new PathMovement(List.of(
                new PathStep(new Node(0, 64, 0), BuiltinMovementType.WALK),
                new PathStep(new Node(1, 64, 0), BuiltinMovementType.WALK),
                new PathStep(new Node(2, 64, 0), BuiltinMovementType.WALK)));
    }

    private static boolean repathRequested(PathMovement movement) throws Exception {
        Field field = PathMovement.class.getDeclaredField("repathRequested");
        field.setAccessible(true);
        return field.getBoolean(movement);
    }

    @Test
    void blockChangeFarFromTheRouteDoesNotRequestRepath() throws Exception {
        PathMovement movement = straightPath();
        movement.requestRepath(40, 63, 12);
        movement.requestRepath(1, 70, 0);
        movement.requestRepath(1, 60, 0);
        movement.requestRepath(4, 64, 0);
        assertFalse(repathRequested(movement));
    }

    @Test
    void blockChangeOnOrBesideTheRouteRequestsRepath() throws Exception {
        PathMovement onRoute = straightPath();
        onRoute.requestRepath(2, 64, 0);
        assertTrue(repathRequested(onRoute));

        PathMovement floor = straightPath();
        floor.requestRepath(1, 63, 0);
        assertTrue(repathRequested(floor));

        PathMovement beside = straightPath();
        beside.requestRepath(3, 65, 1);
        assertTrue(repathRequested(beside));
    }
}
