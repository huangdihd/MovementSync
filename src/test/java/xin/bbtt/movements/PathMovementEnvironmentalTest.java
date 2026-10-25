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

    @Test
    void replannedSwimRecoveryAscendsBeforeGlidingMetadataClears() throws Exception {
        var sync = new MovementSync();
        World world = poolWorld(false);
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(sync, world);
        sync.position.set(new Vector3d(3.5, 1.30, .5));
        sync.velocity.set(new Vector3d(0, -.04, 0));
        sync.onGround.set(false);
        sync.setGliding(true);
        var goal = new Vector3i(8, 4, 0);
        long generation = sync.beginStaticNavigationRequest(goal);
        var movement = new PathMovement(List.of(), 1, false, goal, -1, generation);
        movement.requestRepath();
        movement.onTick();
        movement.onTick();
        assertTrue(sync.getInput().jump(), "valid upward water recovery must not be rejected as a ground-floor deviation");
        assertTrue(sync.isGliding(), "swimming does not invent metadata clearance");
        var pose = xin.bbtt.physics.PlayerPose.GLIDING;
        for (int tick = 0; tick < 12; tick++) {
            movement.onTick();
            var result = new xin.bbtt.physics.GlidingPhysics(sync.yaw.get(), sync.pitch.get(), false,
                    sync.getInput(), xin.bbtt.physics.MovementParameters.DEFAULT).step(world,
                    new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), 0, pose),
                    xin.bbtt.physics.PlayerPose.HALF_WIDTH, pose.height());
            sync.position.set(result.position());
            sync.velocity.set(result.velocity());
            sync.onGround.set(result.onGround());
            pose = result.pose();
        }
        assertTrue(sync.position.get().y > 1.60, "recovery must physically ascend before metadata changes");
        assertTrue(sync.velocity.get().y > 0);
        assertTrue(sync.isGliding());
        movement.onStop();
        assertEquals(PlayerInput.NONE, sync.getInput());
    }

    @Test
    void partialSwimEndpointWaitsForGlidingMetadataBeforeReplanning() throws Exception {
        var sync = new MovementSync();
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                return x >= (29047 >> 4) && x <= (29047 >> 4) + 1 && z == (43 >> 4);
            }
            @Override public int getBlockAt(Vector3d position) {
                return position.y < 60 ? 1 : position.y < 63 ? 86 : 0;
            }
        };
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(sync, world);
        var endpoint = new Node(29047, 63, 43);
        Vector3d surface = xin.bbtt.pathfinding.EnvironmentalTraversal.position(world, endpoint).orElseThrow();
        sync.position.set(new Vector3d(surface));
        sync.velocity.set(new Vector3d());
        sync.onGround.set(false);
        sync.setGliding(true);
        var goal = new Vector3i(29950, 70, 43);
        long generation = sync.beginStaticNavigationRequest(goal);
        // The remaining suffix of a budget-exhausted route is not the requested goal.
        List<PathStep> partial = List.of(new PathStep(endpoint, BuiltinMovementType.SWIM));
        var movement = new PathMovement(partial, 1, false, goal, -1, generation);
        var clock = new java.util.concurrent.atomic.AtomicLong();
        Field watchdogField = PathMovement.class.getDeclaredField("progressWatchdog");
        watchdogField.setAccessible(true);
        watchdogField.set(movement, new PathProgressWatchdog(200, clock::get));
        assertTrue(xin.bbtt.pathfinding.EnvironmentalTraversal.inWater(world, surface));
        movement.onTick();
        Field pathField = PathMovement.class.getDeclaredField("path");
        pathField.setAccessible(true);
        assertSame(partial, pathField.get(movement),
                "water contact while still gliding is not a supported node-arrival proof");
        assertFalse(movement.isFinished(), "a partial endpoint is not arrival at the distant goal");
        assertTrue(sync.isGliding(), "path execution must not forge metadata clearance");
        sync.setGliding(false);
        movement.onTick();
        assertNotSame(partial, pathField.get(movement),
                "the supported endpoint may replan only after authoritative glide metadata clears");
        var pose = xin.bbtt.physics.PlayerPose.STANDING;
        for (int tick = 0; tick < 240 && !movement.isFinished(); tick++) {
            clock.addAndGet(50_000_000L);
            movement.onTick();
            var result = new xin.bbtt.physics.GlidingPhysics(sync.yaw.get(), sync.pitch.get(), false,
                    sync.getInput(), xin.bbtt.physics.MovementParameters.DEFAULT).step(world,
                    new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), 0, pose),
                    xin.bbtt.physics.PlayerPose.HALF_WIDTH, pose.height());
            sync.position.set(result.position());
            sync.velocity.set(result.velocity());
            sync.onGround.set(result.onGround());
            pose = result.pose();
        }
        assertFalse(movement.isFinished(), "water-supported progress must not end in NO_PROGRESS");
        assertNull(movement.failureReason());
        assertTrue(sync.position.get().x > surface.x + 5, "must physically continue toward the distant goal");
        assertEquals(goal, sync.getActiveGoal());
        assertFalse(sync.isGliding());
        movement.onStop();
        assertEquals(PlayerInput.NONE, sync.getInput());
    }

    @Test
    void dryAirborneFinalGoalStillRequiresLanding() throws Exception {
        var sync = new MovementSync();
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d position) { return position.y < 1 ? 1 : 0; }
        };
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(sync, world);
        var goal = new Vector3i(3, 1, 0);
        long generation = sync.beginStaticNavigationRequest(goal);
        sync.position.set(new Vector3d(3.5, 1.2, .5));
        sync.velocity.set(new Vector3d(0, -.04, 0));
        sync.onGround.set(false);
        sync.setGliding(true);
        var movement = new PathMovement(List.of(new PathStep(new Node(3, 1, 0),
                BuiltinMovementType.WALK)), 1, false, goal, -1, generation);
        assertFalse(xin.bbtt.pathfinding.EnvironmentalTraversal.inWater(world, sync.position.get()));
        movement.onTick();
        assertFalse(movement.isFinished(), "dry gliding inside the arrival tolerance is not landing");
        assertEquals(goal, sync.getActiveGoal());
        sync.setGliding(false);
        movement.onTick();
        assertFalse(movement.isFinished(), "clearing metadata alone is not physical support");
        assertEquals(goal, sync.getActiveGoal());
        sync.position.set(new Vector3d(3.5, 1, .5));
        sync.onGround.set(true);
        movement.onTick();
        assertTrue(movement.isFinished());
        assertEquals(PathMovement.Completion.NORMAL, movement.completion());
        assertNull(sync.getActiveGoal());
    }

    @Test
    void fullSearchKeepsSurfaceAfloatWhileGoalRemainsUnknown() throws Exception {
        var fixture = new PlanningFixture(true, false, 62.64);
        fixture.world.duringSearch = () -> {
            assertNavigationLockAvailable(fixture.sync, fixture.generation);
            fixture.physics(80);
            assertTrue(fixture.sync.position.get().y >= 62.46,
                    "physics during full search must not sink below the surface: " + fixture.sync.position.get());
            assertEquals(PlayerInput.NONE.withJump(true), fixture.sync.getInput());
        };
        fixture.movement.requestRepath();
        fixture.movement.onTick();
        assertEquals(1, fixture.world.searchCallbacks);
        assertTrue((boolean) field("waitingForRoute").get(fixture.movement));
        assertFalse(fixture.movement.isFinished(), "water support is not arrival at the unknown distant goal");
        assertEquals(fixture.goal, fixture.sync.getActiveGoal());
        assertTrue(fixture.sync.isGliding(), "planning must not forge gliding metadata");
        assertVerticalPlanningInput(fixture.sync);
        fixture.movement.onStop();
        assertEquals(PlayerInput.NONE, fixture.sync.getInput());
    }

    @Test
    void fullSearchKeepsOpenColumnRecoveryAscending() throws Exception {
        var fixture = new PlanningFixture(true, false, 54.3);
        fixture.world.duringSearch = () -> {
            fixture.physics(40);
            assertTrue(fixture.sync.position.get().y > 54.6,
                    "open-column ascent must continue while the search runs");
            assertTrue(fixture.sync.velocity.get().y > 0);
            assertEquals(PlayerInput.NONE.withJump(true), fixture.sync.getInput());
        };
        fixture.movement.requestRepath();
        fixture.movement.onTick();
        assertEquals(1, fixture.world.searchCallbacks);
        assertTrue(fixture.sync.isGliding());
        fixture.movement.onStop();
        assertEquals(PlayerInput.NONE, fixture.sync.getInput());
    }

    @Test
    void waitingForEmptyRouteRefreshesVerticalWaterSafety() throws Exception {
        for (double y : new double[]{62.64, 54.3}) {
            var fixture = new PlanningFixture(true, false, y);
            // The empty-search result is the retry boundary; physics continues between retries.
            var apply = PathMovement.class.getDeclaredMethod("applyRepathResult", List.class, Node.class);
            apply.setAccessible(true);
            fixture.sync.runIfNavigationRequestCurrent(fixture.generation,
                    () -> {
                        try { apply.invoke(fixture.movement, List.of(), new Node(29950, 63, 43)); }
                        catch (ReflectiveOperationException error) { throw new AssertionError(error); }
                    });
            field("ticksSinceRepath").set(fixture.movement, 0);
            for (int tick = 0; tick < 19; tick++) {
                fixture.movement.onTick();
                assertVerticalPlanningInput(fixture.sync);
                fixture.physics(1);
            }
            assertTrue(fixture.sync.position.get().y >= y - .18);
            if (y < 60) assertTrue(fixture.sync.position.get().y > y + .3);
            assertFalse(fixture.movement.isFinished());
            assertEquals(fixture.goal, fixture.sync.getActiveGoal());
            assertTrue(fixture.sync.isGliding());
            fixture.movement.onStop();
            assertEquals(PlayerInput.NONE, fixture.sync.getInput());
        }
    }

    @Test
    void dryAndRoofedPlanningRemainNeutral() throws Exception {
        for (boolean water : new boolean[]{false, true}) {
            var fixture = new PlanningFixture(water, water, water ? 54.3 : 63);
            fixture.world.duringSearch = () -> assertEquals(PlayerInput.NONE, fixture.sync.getInput(),
                    "dry terrain or a blocked water column must not receive blind jump input");
            fixture.movement.requestRepath();
            fixture.movement.onTick();
            assertEquals(1, fixture.world.searchCallbacks);
            assertEquals(PlayerInput.NONE, fixture.sync.getInput());
            fixture.movement.onStop();
        }
    }

    @Test
    void stoppedSearchCannotReissueFloatInput() throws Exception {
        var fixture = new PlanningFixture(true, false, 62.64);
        fixture.world.duringSearch = () -> {
            assertEquals(PlayerInput.NONE.withJump(true), fixture.sync.getInput());
            fixture.movement.onStop();
            assertEquals(PlayerInput.NONE, fixture.sync.getInput());
        };
        fixture.movement.requestRepath();
        fixture.movement.onTick();
        fixture.movement.onTick();
        assertEquals(1, fixture.world.searchCallbacks);
        assertEquals(PlayerInput.NONE, fixture.sync.getInput());
    }

    @Test
    void supersededSearchReleasesOnlyItsOwnFloatInput() throws Exception {
        for (boolean replacementInput : new boolean[]{false, true}) {
            var fixture = new PlanningFixture(true, false, 62.64);
            var replacement = new PlayerInput(true, false, false, false, false, false, false);
            fixture.world.duringSearch = () -> {
                assertEquals(PlayerInput.NONE.withJump(true), fixture.sync.getInput());
                fixture.sync.beginStaticNavigationRequest(new Vector3i(29960, 70, 43));
                if (replacementInput) fixture.sync.setInput(replacement);
            };
            fixture.movement.requestRepath();
            fixture.movement.onTick();
            assertEquals(1, fixture.world.searchCallbacks);
            assertTrue(fixture.movement.isFinished());
            assertSame(replacementInput ? replacement : PlayerInput.NONE, fixture.sync.getInput());
            fixture.movement.onStop();
            assertSame(replacementInput ? replacement : PlayerInput.NONE, fixture.sync.getInput());
        }
    }

    private static void assertVerticalPlanningInput(MovementSync sync) {
        assertEquals(PlayerInput.NONE, sync.getInput().withJump(false),
                "planning must never issue horizontal/sneak/sprint input");
        if (xin.bbtt.pathfinding.EnvironmentalTraversal.inWater(sync.getWorld(), sync.position.get())) {
            assertTrue(sync.getInput().jump(), "water contact requires float/ascent input");
        }
    }

    private static Field field(String name) throws Exception {
        Field field = PathMovement.class.getDeclaredField(name);
        field.setAccessible(true);
        return field;
    }

    private static void assertNavigationLockAvailable(MovementSync sync, long generation) {
        var executor = java.util.concurrent.Executors.newSingleThreadExecutor();
        try {
            assertTrue(executor.submit(() -> sync.runIfNavigationRequestCurrent(generation, () -> {}))
                    .get(2, java.util.concurrent.TimeUnit.SECONDS), "search must not hold the navigation lock");
        } catch (Exception error) {
            throw new AssertionError("navigation lock unavailable during full search", error);
        } finally {
            executor.shutdownNow();
        }
    }

    private static final class PlanningFixture {
        final MovementSync sync = new MovementSync();
        final Vector3i goal = new Vector3i(30000, 70, 43);
        final PlanningWorld world;
        final long generation;
        final PathMovement movement;
        xin.bbtt.physics.PlayerPose pose = xin.bbtt.physics.PlayerPose.GLIDING;

        PlanningFixture(boolean water, boolean roof, double y) throws Exception {
            world = new PlanningWorld(sync, water, roof);
            Field worldField = MovementSync.class.getDeclaredField("world");
            worldField.setAccessible(true);
            worldField.set(sync, world);
            sync.position.set(new Vector3d(29950.5, y, 43.5));
            sync.velocity.set(new Vector3d());
            sync.onGround.set(!water);
            sync.setGliding(true);
            generation = sync.beginStaticNavigationRequest(goal);
            movement = new PathMovement(List.of(), 1, false, goal, -1, generation);
            // Replanning must replace owned movement keys, not merely add jump to them.
            var previous = new PlayerInput(true, false, false, false, false, true, true);
            field("issuedInput").set(movement, previous);
            sync.setInput(previous);
        }

        void physics(int ticks) {
            for (int tick = 0; tick < ticks; tick++) {
                assertEquals(PlayerInput.NONE, sync.getInput().withJump(false));
                var result = new xin.bbtt.physics.GlidingPhysics(sync.yaw.get(), sync.pitch.get(), false,
                        sync.getInput(), xin.bbtt.physics.MovementParameters.DEFAULT).step(world,
                        new PhysicsState(sync.position.get(), sync.velocity.get(), sync.onGround.get(), 0, pose),
                        xin.bbtt.physics.PlayerPose.HALF_WIDTH, pose.height());
                sync.position.set(result.position());
                sync.velocity.set(result.velocity());
                sync.onGround.set(result.onGround());
                pose = result.pose();
            }
        }
    }

    /** An isolated, deep open column with a still-unloaded distant destination. */
    private static final class PlanningWorld extends World {
        final MovementSync sync;
        final boolean water;
        final boolean roof;
        Runnable duringSearch;
        int searchCallbacks;

        PlanningWorld(MovementSync sync, boolean water, boolean roof) {
            this.sync = sync;
            this.water = water;
            this.roof = roof;
        }

        @Override public boolean chunkLoaded(int x, int z) {
            // Suspend the actual full search, not the preceding local water validation.
            if (duringSearch != null && StackWalker.getInstance().walk(frames -> frames
                    .anyMatch(frame -> frame.getClassName().equals("xin.bbtt.pathfinding.DStarLite")))) {
                assertFalse(Thread.holdsLock(sync), "full search must not hold the sync monitor");
                Runnable callback = duringSearch;
                duringSearch = null;
                searchCallbacks++;
                callback.run();
            }
            return x != (30000 >> 4);
        }

        @Override public int getBlockAt(Vector3d position) {
            if ((int) Math.floor(position.x) != 29950 || (int) Math.floor(position.z) != 43) return 1;
            if (position.y < 50 || roof && position.y >= 60) return 1;
            return position.y < 63 ? water ? 86 : 1 : 0;
        }
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
        List<PathStep> path = new xin.bbtt.pathfinding.DStarLite(source, goal, context).findPath(2000);
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
