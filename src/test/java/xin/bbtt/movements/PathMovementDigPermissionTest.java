package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;

final class PathMovementDigPermissionTest {
    @Test
    void failClosedPathRefusesPreplannedDigStepAtDispatch() {
        MovementSync movementSync = new MovementSync();
        movementSync.position.set(new Vector3d(0.5, 64.0, 0.5));
        movementSync.onGround.set(true);
        movementSync.setActiveGoal(new Vector3i(1, 64, 0));
        PathMovement movement = new PathMovement(
            List.of(
                new PathStep(new Node(0, 64, 0), BuiltinMovementType.WALK),
                new PathStep(new Node(1, 64, 0), BuiltinMovementType.DIG)
            ),
            false
        );

        movement.init();
        movement.onTick();

        assertTrue(movement.isFinished(), "a no-dig path must abort before dispatching DIG");
        assertNull(movementSync.getActiveGoal(), "the rejected destructive path must clear its goal");
    }
}
