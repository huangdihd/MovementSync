package xin.bbtt.movements;

import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.pathfinding.DefaultPathfindingContext;
import xin.bbtt.pathfinding.DStarLite;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

import java.lang.reflect.Field;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.jupiter.api.Assertions.*;

final class PathMovementPartialOutcomeTest {
    private static final BlockState STONE = new BlockState(
            "minecraft:stone", 1, Map.of(), "block", 1.5, true, "STONE");
    private static final BlockState AIR = new BlockState(
            "minecraft:air", 0, Map.of(), "empty", 0.0, false, "AIR");

    private static final class LoadingWorld extends World {
        int loadedThrough;
        @Override public boolean chunkLoaded(int x, int z) {
            return x >= -1 && x <= loadedThrough && z >= -1 && z <= 0;
        }
        @Override public BlockState getBlockStateAt(Vector3d position) {
            return Math.floor(position.y) == 63 ? STONE : AIR;
        }
    }

    @Test
    void longWalkingRouteWaitsAtPartialEndsAndContinuesWhenChunksLoad() throws Exception {
        LoadingWorld world = new LoadingWorld();
        MovementSync sync = ready(world);
        Vector3i goal = new Vector3i(128, 64, 0);
        sync.setActiveGoal(goal);
        var initial = new DStarLite(new Node(0, 64, 0), new Node(goal.x, goal.y, goal.z),
                new DefaultPathfindingContext(world, false)).findPath(2000);
        assertTrue(initial.size() > 1);
        assertNotEquals(new Node(goal.x, goal.y, goal.z), initial.get(initial.size() - 1).getNode());
        PathMovement movement = new PathMovement(initial);
        AtomicLong clock = installClock(movement);
        int waitingTicks = 0;
        int loads = 0;
        for (int tick = 0; tick < 3000 && !movement.isFinished(); tick++) {
            clock.addAndGet(50_000_000L);
            movement.onTick();
            if (waiting(movement)) {
                assertFalse(movement.isFinished(), "partial frontier is not arrival");
                assertEquals(goal, sync.getActiveGoal());
                assertEquals(PlayerInput.NONE, sync.getInput());
                if (++waitingTicks == 25) {
                    world.loadedThrough++;
                    loads++;
                    waitingTicks = 0;
                    movement.requestRepath();
                }
            }
            if (movement.isFinished()) break;
            var result = new WalkingPhysics(sync.getInput(), sync.yaw.get()).step(world,
                    new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get()),
                    (double) 0.6f / 2, 1.8);
            sync.position.set(result.position());
            sync.velocity.set(result.velocity());
            sync.onGround.set(result.onGround());
        }
        assertTrue(loads >= 8, "must walk through multiple dynamically loaded frontiers");
        assertTrue(movement.isFinished(), "position=" + sync.position.get());
        assertEquals(PathMovement.Completion.NORMAL, movement.completion());
        assertNull(movement.failureReason());
        assertTrue(Math.abs(sync.position.get().x - 128.5) < 0.5);
        assertNull(sync.getActiveGoal());
        assertEquals(PlayerInput.NONE, sync.getInput());
    }

    @Test
    void exhaustedFrontierAbortsAfter200ExecutionTicksWithoutReplanReset() throws Exception {
        LoadingWorld world = new LoadingWorld();
        MovementSync sync = ready(world);
        sync.position.set(new Vector3d(15.5, 64, 0.5));
        Vector3i goal = new Vector3i(128, 64, 0);
        sync.setActiveGoal(goal);
        PathMovement movement = new PathMovement(java.util.List.of());
        AtomicLong clock = installClock(movement);
        movement.init();
        movement.onTick();
        assertFalse(movement.isFinished());
        for (int tick = 1; tick <= 200; tick++) {
            clock.addAndGet(50_000_000L);
            movement.requestRepath();
            movement.onTick();
            if (tick < 200) {
                assertFalse(movement.isFinished(), "premature abort at tick " + tick);
                assertEquals(goal, sync.getActiveGoal());
            }
        }
        assertTrue(movement.isFinished());
        assertEquals(PathMovement.Completion.FAILED, movement.completion());
        assertEquals(PathMovement.FailureReason.NO_PROGRESS, movement.failureReason());
        assertEquals(PlayerInput.NONE, sync.getInput());
        assertNull(sync.getActiveGoal());
        assertEquals(new Vector3d(15.5, 64, 0.5), sync.position.get());
    }

    @Test
    void abortWarningIncludesExecutionAndShoreContextInTheSameEvent() throws Exception {
        BlockState lava = new BlockState("minecraft:lava", 2, Map.of(), "empty", 100.0, false, "LAVA");
        World world = new World() {
            @Override public BlockState getBlockStateAt(Vector3d position) {
                if (Math.floor(position.y) != 32) return AIR;
                return Math.floor(position.x) >= 31 ? lava : STONE;
            }
        };
        MovementSync sync = ready(world);
        sync.position.set(new Vector3d(30.7, 33, -.4996));
        sync.velocity.set(new Vector3d(.02, -.08, .01));
        sync.onGround.set(false);
        sync.setActiveGoal(new Vector3i(40, 33, -1));
        var next = new xin.bbtt.pathfinding.PathStep(new Node(31, 33, -1),
                xin.bbtt.pathfinding.BuiltinMovementType.WALK);
        PathMovement movement = new PathMovement(java.util.List.of(next));
        assertAbortContext(movement, sync, "next=" + next, "onGround=false", "31/32/-1=minecraft:lava");
    }

    @Test
    void abortWarningIncludesContextWithNoRemainingRoute() throws Exception {
        MovementSync sync = ready(new LoadingWorld());
        sync.setActiveGoal(new Vector3i(128, 64, 0));
        PathMovement movement = new PathMovement(java.util.List.of());
        assertAbortContext(movement, sync, "next=null", "step=0/0", "0/63/0=minecraft:stone");
    }

    private static void assertAbortContext(PathMovement movement, MovementSync sync,
                                          String... expected) throws Exception {
        var logger = (ch.qos.logback.classic.Logger) MovementSync.getLogger();
        var previousLevel = logger.getLevel();
        var messages = new ch.qos.logback.core.read.ListAppender<ch.qos.logback.classic.spi.ILoggingEvent>();
        messages.start();
        logger.addAppender(messages);
        logger.setLevel(ch.qos.logback.classic.Level.WARN);
        String velocity = sync.velocity.get().toString();
        try {
            var abort = PathMovement.class.getDeclaredMethod("abortLivelockedGoal", Vector3d.class);
            abort.setAccessible(true);
            abort.invoke(movement, sync.position.get());
            var warnings = messages.list.stream()
                    .filter(event -> event.getFormattedMessage().startsWith("Pathfinding aborted:"))
                    .toList();
            assertEquals(1, warnings.size());
            assertEquals(ch.qos.logback.classic.Level.WARN, warnings.get(0).getLevel());
            String warning = warnings.get(0).getFormattedMessage();
            for (String field : new String[]{"position=", "goal=", "currentNode=", "previous=",
                    "velocity=" + velocity, "onGround=", "feetBlock=", "belowBlock=",
                    "nearbySupport=[", "waitingForRoute="}) {
                assertTrue(warning.contains(field), warning);
            }
            for (String field : expected) assertTrue(warning.contains(field), warning);
            assertEquals(PathMovement.Completion.FAILED, movement.completion());
            assertEquals(PathMovement.FailureReason.NO_PROGRESS, movement.failureReason());
        } finally {
            logger.detachAppender(messages);
            logger.setLevel(previousLevel);
            messages.stop();
        }
    }

    private static MovementSync ready(World world) throws Exception {
        MovementSync sync = new MovementSync();
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(sync, world);
        sync.position.set(new Vector3d(0.5, 64, 0.5));
        sync.velocity.set(new Vector3d());
        sync.onGround.set(true);
        sync.setFollowTargetId(-1);
        return sync;
    }

    private static AtomicLong installClock(PathMovement movement) throws Exception {
        AtomicLong clock = new AtomicLong();
        Field field = PathMovement.class.getDeclaredField("progressWatchdog");
        field.setAccessible(true);
        field.set(movement, new PathProgressWatchdog(200, clock::get));
        return clock;
    }

    private static boolean waiting(PathMovement movement) throws Exception {
        Field field = PathMovement.class.getDeclaredField("waitingForRoute");
        field.setAccessible(true);
        return field.getBoolean(movement);
    }
}
