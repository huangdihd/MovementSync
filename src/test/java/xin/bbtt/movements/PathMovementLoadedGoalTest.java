package xin.bbtt.movements;

import java.lang.reflect.Field;
import java.util.List;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerState;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.PathStep;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

class PathMovementLoadedGoalTest {
    @AfterEach void reset() { MovementSync.INSTANCE = null; }

    @Test void newlyLoadedSuspendedGoalFailsAndReleasesSwimmingInput() throws Exception {
        exercise(true, false);
    }

    @Test void newlyLoadedSuspendedGoalFailsWithoutSteeringAuthoritativeGlide() throws Exception {
        exercise(true, true);
    }

    @Test void unknownGoalRemainsPendingRatherThanBeingDeclaredInvalid() throws Exception {
        exercise(false, false);
    }

    @Test void invalidGoalPredicateUsesOneTerrainViewAndReleasesItBeforePublishing() throws Exception {
        var readingTerrain = new java.util.concurrent.atomic.AtomicBoolean();
        var terrainQueries = new java.util.concurrent.atomic.AtomicInteger();
        var blockReads = new java.util.concurrent.atomic.AtomicInteger();
        var sync = new MovementSync() {
            @Override public boolean runIfNavigationRequestCurrent(long generation, Runnable action) {
                assertFalse(readingTerrain.get(), "release terrain lock before acquiring navigation lock");
                return super.runIfNavigationRequestCurrent(generation, action);
            }
        };
        World world = new World() {
            @Override public boolean readTerrain(java.util.function.BooleanSupplier query) {
                terrainQueries.incrementAndGet();
                return super.readTerrain(() -> {
                    readingTerrain.set(true);
                    try { return query.getAsBoolean(); }
                    finally { readingTerrain.set(false); }
                });
            }
            @Override public boolean chunkLoaded(int x, int z) {
                assertTrue(readingTerrain.get(), "loaded check must share the terrain view with block checks");
                return true;
            }
            @Override public int getBlockAt(Vector3d p) {
                if (p.x >= 16) {
                    assertTrue(readingTerrain.get(), "goal block checks must remain inside the same terrain view");
                    blockReads.incrementAndGet();
                    return 0;
                }
                // Any unrelated player-position read must remain outside the goal query.
                assertFalse(readingTerrain.get(), "non-goal reads are outside the goal query");
                return p.y < 64 ? 1 : 0;
            }
        };
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(sync, world);
        sync.position.set(new Vector3d(8.5, 64, 8.5));
        var goal = new Vector3i(32, 70, 0);
        long generation = sync.beginStaticNavigationRequest(goal);
        var movement = new PathMovement(List.of(), 1, false, goal, -1, generation);
        // Isolate the compound invalid-goal query from unrelated per-tick terrain reads.
        var computeRepath = PathMovement.class.getDeclaredMethod("computeRepath");
        computeRepath.setAccessible(true);
        computeRepath.invoke(movement);
        assertEquals(1, terrainQueries.get(), "loaded and support checks use one short query");
        assertTrue(blockReads.get() > 0, "the compound query must inspect goal support");
        assertTrue(movement.isFinished());
        assertEquals(PathMovement.Completion.FAILED, movement.completion());
        assertEquals(PathMovement.FailureReason.INVALID_GOAL, movement.failureReason());
        assertNull(sync.getActiveGoal());
        assertEquals(PlayerInput.NONE, sync.getInput());
    }

    @Test void finalWaterGoalWaitsForAuthoritativeGlideMetadataToClear() throws Exception {
        var sync = new MovementSync();
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d p) { return p.y < 60 ? 1 : p.y < 63 ? 86 : 0; }
        };
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(sync, world);
        var goal = new Vector3i(3, 63, 0);
        var surface = xin.bbtt.pathfinding.EnvironmentalTraversal.position(
                world, new Node(goal)).orElseThrow();
        sync.position.set(surface);
        sync.velocity.set(new Vector3d());
        sync.onGround.set(false);
        sync.setGliding(true);
        long generation = sync.beginStaticNavigationRequest(goal);
        var movement = new PathMovement(List.of(new PathStep(new Node(goal),
                BuiltinMovementType.SWIM)), 1, false, goal, -1, generation);
        movement.onTick();
        assertFalse(movement.isFinished(),
                "water contact while authoritative glide metadata remains is not arrival");
        assertEquals(goal, sync.getActiveGoal());
        assertTrue(sync.isGliding());
        sync.setGliding(false);
        movement.onTick();
        assertTrue(movement.isFinished(), "the same water-supported goal may complete after metadata clears");
        assertEquals(PathMovement.Completion.NORMAL, movement.completion());
        assertNull(sync.getActiveGoal());
        assertEquals(PlayerInput.NONE, sync.getInput());
    }

    private void exercise(boolean loadGoal, boolean gliding) throws Exception {
        var sync = new MovementSync();
        var loaded = new java.util.concurrent.atomic.AtomicBoolean();
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                return x <= 0 || loaded.get() && x == 2;
            }
            @Override public int getBlockAt(Vector3d p) {
                return p.y < 58 ? 1 : p.y < 63 ? 86 : 0;
            }
        };
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(sync, world);
        sync.position.set(new Vector3d(8.5, 62.64, 8.5));
        sync.velocity.set(new Vector3d(.1, -.2, .3));
        sync.onGround.set(false);
        sync.setGliding(gliding);
        sync.yaw.set(37f);
        sync.pitch.set(-12f);
        if (gliding) {
            sync.queueBeforeMovement(new ServerboundPlayerCommandPacket(42, PlayerState.START_ELYTRA_FLYING));
            sync.queueBeforeMovement(new ServerboundUseItemPacket(Hand.MAIN_HAND, 1, 0, 0));
        }
        var goal = new Vector3i(32, 70, 0);
        long generation = sync.beginStaticNavigationRequest(goal);
        var movement = new PathMovement(List.of(new PathStep(new Node(8, 63, 8),
                BuiltinMovementType.SWIM)), 1, false, goal, -1, generation);
        assertFalse(world.chunkLoaded(goal.x >> 4, goal.z >> 4));
        loaded.set(loadGoal);
        assertEquals(loadGoal, world.chunkLoaded(goal.x >> 4, goal.z >> 4));
        assertTrue(xin.bbtt.pathfinding.StandablePositionResolver.feetY(world, 32, 70, 0).isEmpty());
        assertTrue(xin.bbtt.pathfinding.EnvironmentalTraversal.position(world, new Node(goal)).isEmpty());
        assertTrue(xin.bbtt.pathfinding.EnvironmentalTraversal.inWater(world, sync.position.get()));
        movement.requestRepath();
        movement.onTick();
        if (loadGoal) {
            assertTrue(movement.isFinished(), "loaded unsupported destination must stop retrying");
            assertEquals(PathMovement.Completion.FAILED, movement.completion());
            assertEquals(PathMovement.FailureReason.INVALID_GOAL, movement.failureReason());
            assertNull(sync.getActiveGoal());
            assertEquals(PlayerInput.NONE, sync.getInput(), "failure releases buoyancy input");
            assertTrue(sync.getMovementController().getMovements().isEmpty(),
                    "failure in water must not install automatic flotation or glide steering");
            assertEquals(gliding, sync.isGliding(), "failure must not forge authoritative metadata");
            if (gliding) assertNull(sync.pollBeforeMovementPacket(),
                    "terminal failure must clear stale glide-start and rocket actions");
            assertEquals(new Vector3d(.1, -.2, .3), sync.velocity.get(),
                    "failure leaves natural motion untouched");
            assertEquals(37f, sync.yaw.get());
            assertEquals(-12f, sync.pitch.get());
        } else {
            assertFalse(movement.isFinished());
            assertEquals(goal, sync.getActiveGoal());
        }
        movement.onStop();
    }
}
