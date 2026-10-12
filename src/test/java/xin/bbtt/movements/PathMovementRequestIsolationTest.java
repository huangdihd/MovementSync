package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
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

    @ParameterizedTest
    @ValueSource(ints = {0, 1, 2, 3})
    void legacyPartialPathProgressesTowardActiveGoalOnFirstTick(int constructor) throws Exception {
        MovementSync movementSync = readyToWalk();
        Vector3i goalBeyondPath = new Vector3i(10, 64, 0);
        movementSync.setActiveGoal(goalBeyondPath);
        PathMovement movement = legacyPath(constructor);

        movement.init();
        movement.onTick();

        assertFalse(movement.isFinished(), "a partial path is not a superseded request");
        assertTrue(movementSync.getInput().forward(), "the first tick must request forward movement");
        assertEquals(-90f, movementSync.yaw.get(), "forward must point along the path");
        assertEquals(0.0, movementSync.velocity.get().x, "only the physics tick may accelerate");
        assertEquals(goalBeyondPath, movementSync.getActiveGoal());
    }

    @ParameterizedTest
    @ValueSource(ints = {0, 1, 2, 3})
    void legacyPathWithoutActiveGoalProgressesOnFirstTick(int constructor) throws Exception {
        MovementSync movementSync = readyToWalk();
        movementSync.setActiveGoal(null);
        PathMovement movement = legacyPath(constructor);

        movement.init();
        movement.onTick();

        assertFalse(movement.isFinished(), "a standalone path must preserve its null request goal");
        assertTrue(movementSync.getInput().forward(), "the first tick must request forward movement");
        assertEquals(-90f, movementSync.yaw.get(), "forward must point along the path");
        assertEquals(0.0, movementSync.velocity.get().x, "only the physics tick may accelerate");
        assertEquals(null, movementSync.getActiveGoal());
    }

    @ParameterizedTest
    @ValueSource(ints = {0, 1, 2, 3})
    void legacyPartialPathIsSupersededByLaterGoal(int constructor) throws Exception {
        MovementSync movementSync = readyToWalk();
        movementSync.setActiveGoal(new Vector3i(10, 64, 0));
        PathMovement movement = legacyPath(constructor);
        // Even a replacement matching the partial path endpoint is a different request.
        Vector3i replacementGoal = new Vector3i(2, 64, 0);
        movementSync.setActiveGoal(replacementGoal);

        movement.onTick();

        assertTrue(movement.isFinished());
        assertEquals(replacementGoal, movementSync.getActiveGoal());
        assertEquals(0.0, movementSync.velocity.get().x, "superseded paths must not move");
    }

    @ParameterizedTest
    @ValueSource(ints = {0, 1, 2, 3})
    void legacyPathCopiesMutableActiveGoal(int constructor) throws Exception {
        MovementSync movementSync = readyToWalk();
        Vector3i goal = new Vector3i(10, 64, 0);
        movementSync.setActiveGoal(goal);
        PathMovement movement = legacyPath(constructor);
        goal.set(2, 64, 0);
        movementSync.setActiveGoal(goal);

        movement.onTick();

        assertTrue(movement.isFinished(), "the captured goal must not alias the mutable active goal");
        assertEquals(goal, movementSync.getActiveGoal());
        assertEquals(0.0, movementSync.velocity.get().x);
    }

    @Test
    void partialPathEndpointContinuesTowardCapturedGoal() throws Exception {
        MovementSync movementSync = readyToWalk();
        Vector3i goal = new Vector3i(4, 64, 0);
        movementSync.setActiveGoal(goal);
        PathMovement movement = legacyPath(0);
        movement.onTick();
        movementSync.position.set(new Vector3d(2.5, 64, 0.5));

        movement.onTick();

        assertFalse(movement.isFinished());
        assertEquals(goal, movementSync.getActiveGoal());
        assertEquals(xin.bbtt.physics.PlayerInput.NONE, movementSync.getInput());
        movement.onTick();
        assertTrue(movementSync.getInput().forward());
    }

    @Test
    void synchronousRepathReleasesKeysBeforeReadingWorld() throws Exception {
        MovementSync movementSync = readyToWalk();
        movementSync.setActiveGoal(new Vector3i(4, 64, 0));
        PathMovement movement = legacyPath(0);
        movement.onTick();
        assertTrue(movementSync.getInput().forward());
        java.util.concurrent.atomic.AtomicInteger reads = new java.util.concurrent.atomic.AtomicInteger();
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }

            @Override
            public BlockState getBlockStateAt(Vector3d position) {
                reads.incrementAndGet();
                assertEquals(xin.bbtt.physics.PlayerInput.NONE, movementSync.getInput(),
                        "physics must not keep consuming stale keys during planning");
                return Math.floor(position.y) == 63 ? STONE : AIR;
            }
        });
        Method repath = PathMovement.class.getDeclaredMethod("repathInternally");
        repath.setAccessible(true);
        repath.invoke(movement);
        assertTrue(reads.get() > 0);
        assertFalse(movement.isFinished());
    }

    @Test
    void repeatedInPlaceRepathsStillAbortWithoutChargingPlannerTime() throws Exception {
        MovementSync movementSync = readyToWalk();
        movementSync.setActiveGoal(new Vector3i(4, 64, 0));
        PathMovement movement = legacyPath(0);
        java.util.concurrent.atomic.AtomicLong clock = new java.util.concurrent.atomic.AtomicLong();
        Field watchdogField = PathMovement.class.getDeclaredField("progressWatchdog");
        watchdogField.setAccessible(true);
        watchdogField.set(movement, new PathProgressWatchdog(60, clock::get));
        movement.onTick();
        for (int index = 0; index < 61 && !movement.isFinished(); index++) {
            clock.addAndGet(50_000_000L);
            movement.requestRepath();
            movement.onTick();
        }
        assertTrue(movement.isFinished(), "replanning must not reset execution stall history");
        assertEquals(null, movementSync.getActiveGoal());
        assertEquals(xin.bbtt.physics.PlayerInput.NONE, movementSync.getInput());
    }

    private static MovementSync readyToWalk() throws Exception {
        MovementSync movementSync = new MovementSync();
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, flatWorld());
        movementSync.position.set(new Vector3d(0.5, 64.0, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        movementSync.setFollowTargetId(-1);
        return movementSync;
    }

    private static PathMovement legacyPath(int constructor) {
        List<PathStep> path = List.of(
            new PathStep(new Node(0, 64, 0), BuiltinMovementType.WALK),
            new PathStep(new Node(2, 64, 0), BuiltinMovementType.WALK)
        );
        return switch (constructor) {
            case 0 -> new PathMovement(path);
            case 1 -> new PathMovement(path, true);
            case 2 -> new PathMovement(path, 1.5);
            case 3 -> new PathMovement(path, 1.5, true);
            default -> throw new IllegalArgumentException("Unknown constructor: " + constructor);
        };
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
