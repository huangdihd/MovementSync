package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.*;

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
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

final class PathMovementEnvironmentalTest {
    @AfterEach
    void resetSingleton() { MovementSync.INSTANCE = null; }

    @Test
    void surfacePathExecutesWithoutSprintAndReleasesInput() throws Exception {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) { return position.y < 1 ? 86 : 0; }
        };
        runPath(world, new Vector3d(0.5, 0.64, 0.5), new Node(3, 1, 0), BuiltinMovementType.SWIM);
    }

    @Test
    void swimmingEntersAndExitsASolidShore() throws Exception {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                if (position.y >= 1) return 0;
                return position.x >= 1 && position.x < 4 ? 86 : 1;
            }
        };
        runPath(world, new Vector3d(0.5, 1, 0.5), new Node(1, 1, 0), BuiltinMovementType.SWIM);
        runPath(world, new Vector3d(3.5, 0.64, 0.5), new Node(4, 1, 0), BuiltinMovementType.SWIM);
    }

    @Test
    void ladderAndVineExecuteAscentAndDescentWithoutHorizontalMotion() throws Exception {
        for (int block : new int[]{5519, 8188}) {
            World world = new World() {
                @Override public boolean chunkLoaded(int x, int z) { return true; }
                @Override public int getBlockAt(Vector3d position) {
                    if (position.y < 1) return 1;
                    return position.x >= 0 && position.x < 1 && position.z >= 0 && position.z < 1 ? block : 0;
                }
            };
            runPath(world, new Vector3d(0.5, 1, 0.5), new Node(0, 3, 0), BuiltinMovementType.CLIMB);
            runPath(world, new Vector3d(0.5, 3, 0.5), new Node(0, 1, 0), BuiltinMovementType.CLIMB);
        }
    }

    @Test
    void defaultStrategiesClimbFromGroundOntoUpperLanding() throws Exception {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                if (y < 1 || x == 1 && y < 5) return 1;
                return x == 0 && y <= 5 ? 5519 : 0;
            }
        };
        runPath(world, new Vector3d(-0.5, 1, 0.5), new Node(1, 5, 0), BuiltinMovementType.CLIMB);
    }

    @Test
    void defaultStrategiesCrossDeepPoolWithoutRoutingAlongItsFloor() throws Exception {
        runPath(poolWorld(false), new Vector3d(0.5, 4, 0.5), new Node(8, 4, 0), BuiltinMovementType.SWIM);
    }

    @Test
    void defaultStrategiesRecoverVerticallyFromPoolFloorBeforeCrossing() throws Exception {
        runPath(poolWorld(false), new Vector3d(3.5, 1, 0.5), new Node(8, 4, 0), BuiltinMovementType.SWIM);
    }

    @Test
    void defaultStrategiesPreferExistingBridge() throws Exception {
        runPath(poolWorld(true), new Vector3d(0.5, 4, 0.5), new Node(8, 4, 0), BuiltinMovementType.WALK);
    }

    @Test
    void travelStartsRecoveryWhileSuspendedInWater() throws Exception {
        runPath(poolWorld(false), new Vector3d(3.5, 1.2, 0.5), new Node(8, 4, 0),
                BuiltinMovementType.SWIM, true);
    }

    private static World poolWorld(boolean bridge) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (y < 1) return 1;
                if (y >= 4) return 0;
                if (x <= 0 || x >= 8 || z < -15 || z > 15 || bridge && z == 0) return 1;
                return 86;
            }
        };
    }

    private static void runPath(World world, Vector3d start, Node goal, BuiltinMovementType type) throws Exception {
        runPath(world, start, goal, type, false);
    }

    private static void runPath(World world, Vector3d start, Node goal, BuiltinMovementType type,
            boolean travel) throws Exception {
        MovementSync movementSync = new MovementSync();
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        movementSync.position.set(start);
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(world.isOnGround(start));
        Vector3i requestedGoal = new Vector3i(goal.x, goal.y, goal.z);
        long generation = movementSync.beginStaticNavigationRequest(requestedGoal);
        Node source = new Node((int) Math.floor(start.x),
                xin.bbtt.pathfinding.StandablePositionResolver.nodeY(start.y), (int) Math.floor(start.z));
        var context = new xin.bbtt.pathfinding.PathfindingContextBuilder(world)
                .addDefaultStrategies().build();
        List<PathStep> path = new xin.bbtt.pathfinding.DStarLite(source, goal, context).findPath(200);
        assertEquals(goal, path.get(path.size() - 1).getNode());
        assertTrue(path.stream().anyMatch(step -> step.getType() == type));
        xin.bbtt.movement.Movement movement = travel
                ? new TravelMovement(requestedGoal, generation, false)
                : new PathMovement(path, 1, false, requestedGoal, -1, generation);
        if (type == BuiltinMovementType.WALK) {
            assertTrue(path.stream().allMatch(step -> step.getType() == BuiltinMovementType.WALK));
        }
        for (int tick = 0; tick < 400 && !movement.isFinished(); tick++) {
            movement.onTick();
            if (type != BuiltinMovementType.WALK) assertFalse(movementSync.getInput().sprint());
            if (movement.isFinished()) break;
            var result = new WalkingPhysics(movementSync.getInput(), movementSync.yaw.get()).step(world,
                    new PhysicsState(movementSync.position.get(), movementSync.velocity.get(), movementSync.onGround.get()),
                    (double) 0.6f / 2, 1.8);
            movementSync.position.set(result.position());
            movementSync.velocity.set(result.velocity());
            movementSync.onGround.set(result.onGround());
        }
        assertTrue(movement.isFinished(), "path failed at " + movementSync.position.get());
        assertTrue(Math.abs(movementSync.position.get().x - goal.x - 0.5) < 0.3);
        assertTrue(Math.abs(movementSync.position.get().z - goal.z - 0.5) < 0.3);
        assertTrue(Math.abs(movementSync.position.get().y - goal.y) < 0.6,
                "finishing must not merely report a deviation as success");
        assertEquals(PlayerInput.NONE, movementSync.getInput());
        if (xin.bbtt.pathfinding.EnvironmentalTraversal.position(world, goal).isEmpty()
                && xin.bbtt.pathfinding.StandablePositionResolver.feetY(world, goal.x, goal.y, goal.z).isPresent()) {
            assertTrue(movementSync.onGround.get(), "shore arrival must be supported, not a transient flyby");
            assertFalse(xin.bbtt.pathfinding.EnvironmentalTraversal.inWater(world, movementSync.position.get()));
        }
    }
}
