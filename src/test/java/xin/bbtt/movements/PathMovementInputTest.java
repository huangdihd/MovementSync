package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.List;
import org.geysermc.mcprotocollib.network.ClientSession;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.tasks.updateMotionTask;
import xin.bbtt.world.World;

final class PathMovementInputTest {
    private MovementSync movementSync;
    private updateMotionTask motionTask;

    private enum Release {
        STOP, PAUSE, SUPERSEDE
    }

    @BeforeEach
    void setUp() throws Exception {
        movementSync = new MovementSync();
        movementSync.onRecoveryTeleport();
        movementSync.position.set(new Vector3d(0.5, 1, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        installWorld(0, 0);
        setBotField("running", true);
        Bot.INSTANCE.setServer(Server.Game);
        setBotField("session", Proxy.newProxyInstance(ClientSession.class.getClassLoader(),
                new Class<?>[]{ClientSession.class}, (proxy, method, arguments) -> {
                    if (method.getReturnType() == boolean.class) return true;
                    return null;
                }));
        motionTask = new updateMotionTask();
    }

    @AfterEach
    void tearDown() throws Exception {
        setBotField("running", false);
        setBotField("session", null);
        Bot.INSTANCE.setServer(null);
        MovementSync.INSTANCE = null;
    }

    @Test
    void flatPathAcceleratesThroughPhysicsAndReleasesAtGoal() {
        Vector3i goal = new Vector3i(3, 1, 0);
        PathMovement movement = pathTo(goal, BuiltinMovementType.WALK);

        movement.onTick();

        assertTrue(movementSync.getInput().forward());
        assertEquals(-90f, movementSync.yaw.get());
        assertEquals(new Vector3d(), movementSync.velocity.get(),
                "path steering must not write a horizontal velocity directly");
        motionTask.run();
        assertTrue(movementSync.position.get().x > 0.5,
                "the real physics tick must consume the path's held input");

        runToCompletion(movement, goal);
        settleAtGoal(goal);
    }

    @ParameterizedTest
    @EnumSource(Release.class)
    void lifecycleReleaseClearsOwnedInputWithoutErasingMomentum(Release release) {
        PathMovement movement = startMoving();
        Vector3d velocity = new Vector3d(movementSync.velocity.get());
        assertTrue(velocity.x > 0);

        release(movement, release);

        assertEquals(PlayerInput.NONE, movementSync.getInput());
        assertEquals(velocity, movementSync.velocity.get(),
                "releasing keys must not cancel physical momentum");
        double previousX = movementSync.position.get().x;
        motionTask.run();
        assertTrue(movementSync.position.get().x > previousX);
        assertTrue(movementSync.velocity.get().x < velocity.x,
                "released input should coast under friction, not keep accelerating");
        if (release == Release.SUPERSEDE) assertTrue(movement.isFinished());
        if (release == Release.PAUSE) assertFalse(movement.isFinished());
    }

    @ParameterizedTest
    @EnumSource(Release.class)
    void oldPathReleasePreservesNewActionsEqualButDistinctInput(Release release) {
        PathMovement movement = startMoving();
        if (release == Release.SUPERSEDE) movementSync.beginNavigationRequest();
        PlayerInput previous = movementSync.getInput();
        PlayerInput replacement = new PlayerInput(previous.forward(), previous.backward(),
                previous.left(), previous.right(), previous.jump(), previous.sneak(), previous.sprint());
        movementSync.setInput(replacement);
        Vector3d velocity = new Vector3d(movementSync.velocity.get());

        switch (release) {
            case STOP -> movement.onStop();
            case PAUSE -> movement.onPause();
            case SUPERSEDE -> movement.onTick();
        }

        assertSame(replacement, movementSync.getInput(),
                "input ownership is identity-based even when both actions hold the same keys");
        assertEquals(velocity, movementSync.velocity.get());
        movement.onStop();
        assertSame(replacement, movementSync.getInput(), "late repeated cleanup must also be harmless");
        motionTask.run();
        assertTrue(movementSync.velocity.get().x > velocity.x,
                "the next physics tick must still accelerate using the new action's input");
    }

    @Test
    void jumpsOntoOneBlockLedgeAndReleasesInput() throws Exception {
        installWorld(1, 0);
        Vector3i goal = new Vector3i(1, 2, 0);
        PathMovement movement = pathTo(goal, BuiltinMovementType.JUMP);

        movement.onTick();
        assertTrue(movementSync.getInput().jump());
        assertEquals(new Vector3d(), movementSync.velocity.get());
        motionTask.run();
        assertTrue(movementSync.position.get().y > 1);
        assertFalse(movementSync.onGround.get());

        runToCompletion(movement, goal);
        settleAtGoal(goal);
    }

    @ParameterizedTest
    @ValueSource(ints = {1, 2})
    void sprintJumpsAcrossGapAndReleasesInput(int gapWidth) throws Exception {
        installWorld(0, gapWidth);
        Vector3i goal = new Vector3i(gapWidth + 1, 1, 0);
        PathMovement movement = pathTo(goal, BuiltinMovementType.GAP_JUMP);

        movement.onTick();
        assertTrue(movementSync.getInput().forward());
        assertTrue(movementSync.getInput().jump());
        assertTrue(movementSync.getInput().sprint());
        assertEquals(new Vector3d(), movementSync.velocity.get());
        motionTask.run();
        assertTrue(movementSync.position.get().y > 1);
        assertFalse(movementSync.onGround.get());

        runToCompletion(movement, goal);
        settleAtGoal(goal);
    }

    @Test
    void travelPartialFinalPathReplansAndOnlyFinishesAtRequestedGoal() {
        Vector3i goal = new Vector3i(3, 1, 0);
        long generation = movementSync.beginStaticNavigationRequest(goal);
        PathMovement[] walking = new PathMovement[1];
        TravelMovement travel = new TravelMovement(goal, generation, false, new TravelMovement.Legs() {
            @Override public List<PathStep> plan(Vector3i destination) {
                assertEquals(goal, destination);
                // A nonempty partial can contain only the currently reached start node.
                return List.of(new PathStep(new Node(0, 1, 0), BuiltinMovementType.WALK));
            }
            @Override public xin.bbtt.movement.Movement walk(
                    List<PathStep> path, Vector3i destination, boolean intermediate) {
                assertFalse(intermediate);
                walking[0] = new PathMovement(path, -1, false, destination, -1, generation);
                return walking[0];
            }
            @Override public xin.bbtt.movement.Movement flight() {
                throw new AssertionError("ground travel must not fly");
            }
        });
        travel.onTick();
        assertNotNull(walking[0]);
        travel.onTick(); // Exhaust the partial and invoke the real PathMovement replanner.
        assertFalse(walking[0].isFinished());
        assertFalse(travel.isFinished());
        assertEquals(goal, movementSync.getActiveGoal());

        for (int tick = 0; tick < 100 && !travel.isFinished(); tick++) {
            travel.onTick();
            if (!travel.isFinished()) motionTask.run();
        }
        assertTrue(travel.isFinished());
        assertTrue(walking[0].isFinished());
        assertEquals(goal.x + 0.5, movementSync.position.get().x, 0.3);
        assertEquals(goal.y, movementSync.position.get().y, 0.5);
        assertEquals(goal.z + 0.5, movementSync.position.get().z, 0.3);
        assertNull(movementSync.getActiveGoal());
        assertEquals(PlayerInput.NONE, movementSync.getInput());
    }

    private PathMovement startMoving() {
        PathMovement movement = pathTo(new Vector3i(3, 1, 0), BuiltinMovementType.WALK);
        movement.onTick();
        assertTrue(movementSync.getInput().forward());
        motionTask.run();
        return movement;
    }

    private void release(PathMovement movement, Release release) {
        switch (release) {
            case STOP -> movement.onStop();
            case PAUSE -> movement.onPause();
            case SUPERSEDE -> {
                movementSync.beginNavigationRequest();
                movement.onTick();
            }
        }
    }

    @Test
    void boostedTwoBlockAscentIsNotMistakenForPathDeviation() throws Exception {
        installWorld(2, 0);
        movementSync.getMovementParameterState().updateEffect(
                org.geysermc.mcprotocollib.protocol.data.game.entity.Effect.JUMP_BOOST, 1);
        Vector3i goal = new Vector3i(1, 3, 0);
        PathMovement movement = pathTo(goal, BuiltinMovementType.JUMP);
        movement.onTick();
        assertTrue(movementSync.getInput().jump());
        assertFalse(movement.isFinished());
        runToCompletion(movement, goal);
        settleAtGoal(goal);
    }

    private PathMovement pathTo(Vector3i goal, BuiltinMovementType type) {
        long generation = movementSync.beginStaticNavigationRequest(goal);
        PathMovement movement = new PathMovement(List.of(
                new PathStep(new Node(0, 1, 0), BuiltinMovementType.WALK),
                new PathStep(new Node(goal.x, goal.y, goal.z), type)),
                -1, false, goal, -1, generation);
        movement.init();
        return movement;
    }

    private void runToCompletion(PathMovement movement, Vector3i goal) {
        // Drive the production steering and physics in tick order; never teleport to a node.
        for (int tick = 0; tick < 100 && !movement.isFinished(); tick++) {
            movement.onTick();
            if (!movement.isFinished()) motionTask.run();
        }
        assertTrue(movement.isFinished(),
                () -> "path did not finish at " + movementSync.position.get());
        Vector3d position = movementSync.position.get();
        assertEquals(goal.x + 0.5, position.x, 0.3);
        assertEquals(goal.z + 0.5, position.z, 0.3);
        assertEquals(goal.y, position.y, 0.5);
        assertEquals(PlayerInput.NONE, movementSync.getInput());
        assertNull(movementSync.getActiveGoal());
    }

    private void settleAtGoal(Vector3i goal) {
        // Completion may occur in the air; released input must still permit a safe landing.
        for (int tick = 0; tick < 30; tick++) motionTask.run();
        assertEquals(PlayerInput.NONE, movementSync.getInput());
        assertTrue(movementSync.onGround.get());
        assertEquals(goal.y, movementSync.position.get().y, 1.0e-9);
        assertEquals(goal.x + 0.5, movementSync.position.get().x, 0.5);
        assertEquals(goal.z + 0.5, movementSync.position.get().z, 0.5);
        assertEquals(0, movementSync.velocity.get().x, 1.0e-6);
    }

    private void installWorld(int ledgeHeight, int gapWidth) throws Exception {
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                if (gapWidth > 0 && x >= 1 && x <= gapWidth) return 0;
                int surface = x >= 1 ? ledgeHeight : 0;
                return y <= surface ? 1 : 0;
            }
        };
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(movementSync, world);
    }

    private static void setBotField(String name, Object value) throws Exception {
        Field field = Bot.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(Bot.INSTANCE, value);
    }
}
