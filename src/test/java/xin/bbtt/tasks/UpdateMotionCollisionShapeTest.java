package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3i;
import xin.bbtt.pathfinding.Node;
import xin.bbtt.pathfinding.StandablePositionResolver;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
import org.geysermc.mcprotocollib.network.ClientSession;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;
import xin.bbtt.world.World;

final class UpdateMotionCollisionShapeTest {
    private static final int AIR = 0;
    private static final int STONE = 1;
    private static final int SPRUCE_PLANKS = 16;
    private static final int DIRT_PATH = 14613;
    private static final int WHITE_CARPET = 12694;
    private static final int BOTTOM_STONE_SLAB = 13197;
    private static final int OPEN_NORTH_LEFT_JUNGLE_DOOR = 13995;
    private static final int OPEN_SOUTH_LEFT_SPRUCE_DOOR_LOWER = 13883;
    private static final int OPEN_SOUTH_LEFT_SPRUCE_DOOR_UPPER = 13875;
    private static final int SOUTH_FLOOR_GRINDSTONE = 20571;
    private MovementSync movementSync;
    private updateMotionTask task;
    private Method isPlayerBoxColliding;
    private final List<Object> sentPackets = new ArrayList<>();

    @BeforeEach
    void setUp() throws Exception {
        movementSync = new MovementSync();
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                return x == 0 && (y == 0 || y == 1) && z == 0
                    ? OPEN_NORTH_LEFT_JUNGLE_DOOR
                    : AIR;
            }
        };
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);

        setBotField("running", true);
        Bot.INSTANCE.setServer(Server.Game);
        setBotField("session", Proxy.newProxyInstance(
            ClientSession.class.getClassLoader(),
            new Class<?>[]{ClientSession.class},
            (proxy, method, args) -> {
                if (method.getName().equals("send")) sentPackets.add(args[0]);
                Class<?> returnType = method.getReturnType();
                if (!returnType.isPrimitive()) return null;
                if (returnType == boolean.class) return false;
                if (returnType == byte.class) return (byte) 0;
                if (returnType == short.class) return (short) 0;
                if (returnType == int.class) return 0;
                if (returnType == long.class) return 0L;
                if (returnType == float.class) return 0.0f;
                if (returnType == double.class) return 0.0d;
                if (returnType == char.class) return '\0';
                return null;
            }
        ));

        task = new updateMotionTask();
        isPlayerBoxColliding = updateMotionTask.class.getDeclaredMethod(
            "isPlayerBoxColliding", Vector3d.class);
        isPlayerBoxColliding.setAccessible(true);
    }

    @AfterEach
    void tearDown() throws Exception {
        setBotField("running", false);
        Bot.INSTANCE.setServer(null);
        setBotField("session", null);
        MovementSync.INSTANCE = null;
    }

    private static void setBotField(String name, Object value) throws Exception {
        Field field = Bot.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(Bot.INSTANCE, value);
    }

    private void setWorld(World world) throws Exception {
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
    }

    private boolean collides(Vector3d feetPosition) throws Exception {
        return (boolean) isPlayerBoxColliding.invoke(task, feetPosition);
    }

    private void useTerrain(java.util.function.ToIntFunction<Vector3d> blocks) throws Exception {
        setWorld(new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }

            @Override
            public int getBlockAt(Vector3d position) { return blocks.applyAsInt(position); }
        });
        movementSync.position.set(new Vector3d(0.5, 1, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
    }

    private void assertTick(double x, double y, double z, double velocityX,
            double velocityY, double velocityZ, boolean onGround,
            boolean collision, boolean sendsPacket) {
        sentPackets.clear();
        task.run();
        Vector3d position = movementSync.position.get();
        Vector3d velocity = movementSync.velocity.get();
        assertEquals(x, position.x, 1.0e-9);
        assertEquals(y, position.y, 1.0e-9);
        assertEquals(z, position.z, 1.0e-9);
        assertEquals(velocityX, velocity.x, 1.0e-9);
        assertEquals(velocityY, velocity.y, 1.0e-9);
        assertEquals(velocityZ, velocity.z, 1.0e-9);
        assertEquals(onGround, movementSync.onGround.get());
        assertEquals(sendsPacket ? 1 : 0, sentPackets.size());
        if (!sendsPacket) return;
        var packet = org.junit.jupiter.api.Assertions.assertInstanceOf(
            org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosPacket.class,
            sentPackets.get(0));
        assertEquals(x, packet.getX(), 1.0e-9);
        assertEquals(y, packet.getY(), 1.0e-9);
        assertEquals(z, packet.getZ(), 1.0e-9);
        assertEquals(onGround, packet.isOnGround());
        assertEquals(collision, packet.isHorizontalCollision());
    }

    @Test
    void stationaryAndHorizontalTicksPreservePacketSuppression() throws Exception {
        useTerrain(position -> Math.floor(position.y) == 0 ? STONE : AIR);
        assertTick(0.5, 1, 0.5, 0, 0, 0, true, false, true);
        assertTick(0.5, 1, 0.5, 0, 0, 0, true, false, false);
        movementSync.velocity.set(new Vector3d(0.2, 0, 0.1));
        for (int tick = 1; tick <= 4; tick++) {
            assertTick(0.5 + 0.2 * tick, 1, 0.5 + 0.1 * tick,
                0.2, 0, 0.1, true, false, true);
        }
    }

    @Test
    void fullJumpTraceIncludesVelocityGroundAndPackets() throws Exception {
        useTerrain(position -> Math.floor(position.y) == 0 ? STONE : AIR);
        movementSync.jump();
        double height = 1;
        double speed = 0.42;
        for (int tick = 0; tick < 12; tick++) {
            height += speed;
            speed = (speed - 0.08) * 0.9800000190734863D;
            if (tick == 11) {
                height = 1;
                speed = 0;
            }
            assertTick(0.5, height, 0.5, 0, speed, 0, tick == 11, false, true);
        }
        assertTick(0.5, 1, 0.5, 0, 0, 0, true, false, false);
    }

    @Test
    void walkingOffPlatformDelaysGravityUntilFollowingTick() throws Exception {
        useTerrain(position -> Math.floor(position.y) == 0
            || (Math.floor(position.y) == 1 && Math.floor(position.x) == 0) ? STONE : AIR);
        movementSync.position.set(new Vector3d(0.5, 2, 0.5));
        movementSync.velocity.set(new Vector3d(1, 0, 0));
        assertTick(1.5, 2, 0.5, 1, 0, 0, false, false, true);
        double height = 2;
        double speed = 0;
        for (int tick = 1; tick <= 6; tick++) {
            height += speed;
            speed = (speed - 0.08) * 0.9800000190734863D;
            if (tick == 6) {
                height = 1;
                speed = 0;
            }
            assertTick(1.5 + tick, height, 0.5, 1, speed, 0, tick == 6, false, true);
        }
    }

    @Test
    void wallsStopEachAxisAndReportCollisionTransitions() throws Exception {
        for (boolean xAxis : new boolean[]{true, false}) {
            useTerrain(position -> Math.floor(position.y) == 0
                || Math.floor(xAxis ? position.x : position.z) == 1 ? STONE : AIR);
            task = new updateMotionTask();
            movementSync.velocity.set(xAxis ? new Vector3d(0.3, 0, 0) : new Vector3d(0, 0, 0.3));
            assertTick(0.5, 1, 0.5, 0, 0, 0, true, true, true);
            assertTick(0.5, 1, 0.5, 0, 0, 0, true, false, true);
            assertTick(0.5, 1, 0.5, 0, 0, 0, true, false, false);
        }
    }

    @Test
    void slabStepTracePreservesHorizontalSpeed() throws Exception {
        useTerrain(position -> Math.floor(position.y) == 0 ? STONE
            : Math.floor(position.y) == 1 && Math.floor(position.x) == 1 ? BOTTOM_STONE_SLAB : AIR);
        movementSync.velocity.set(new Vector3d(0.3, 0, 0));
        for (int tick = 1; tick <= 3; tick++) {
            assertTick(0.5 + tick * 0.3, 1.5, 0.5, 0.3, 0, 0, true, false, true);
        }
    }

    @Test
    void ceilingCancelsJumpWithoutHorizontalCollision() throws Exception {
        useTerrain(position -> Math.floor(position.y) == 0 || Math.floor(position.y) == 3 ? STONE : AIR);
        movementSync.jump();
        assertTick(0.5, 1, 0.5, 0, 0, 0, true, false, true);
        assertTick(0.5, 1, 0.5, 0, 0, 0, true, false, false);
    }

    @Test
    void unloadedChunkLeavesStateAndPacketsUntouched() throws Exception {
        setWorld(new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return false; }
        });
        movementSync.position.set(new Vector3d(0.5, 4, 0.5));
        movementSync.velocity.set(new Vector3d(0.2, -0.3, 0.4));
        movementSync.onGround.set(true);
        for (int tick = 0; tick < 3; tick++) {
            assertTick(0.5, 4, 0.5, 0.2, -0.3, 0.4, true, false, false);
        }
    }

    @Test
    void openDoorAllowsCentreApproachButBlocksItsRotatedPanel() throws Exception {
        assertFalse(collides(new Vector3d(0.5, 0.0, 0.5)),
            "player centred in the doorway must clear the west-side panel");
        assertTrue(collides(new Vector3d(0.1, 0.0, 0.5)),
            "player overlapping the west-side panel must collide");
    }

    @Test
    void horizontalMotionUsesVanillaStepHeightFromDirtPathToFullBlock() throws Exception {
        setWorld(new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (y != 0 || z != 0) return AIR;
                if (x == 0) return DIRT_PATH;
                if (x == 1) return STONE;
                return AIR;
            }
        });
        movementSync.position.set(new Vector3d(0.5, 15.0 / 16.0, 0.5));
        movementSync.velocity.set(new Vector3d(MovementSync.movementSpeed, 0, 0));
        movementSync.onGround.set(true);

        task.run();

        Vector3d stepped = movementSync.position.get();
        assertTrue(stepped.x > 0.5, "horizontal motion should continue onto the higher surface");
        assertEquals(1.0, stepped.y, 1.0e-9, "feet should rise by the dirt-path height difference");
    }

    @Test
    void ridingSendsCurrentRotationEveryTick() throws Exception {
        Field vehicleField = MovementSync.class.getDeclaredField("vehicleId");
        vehicleField.setAccessible(true);
        vehicleField.set(movementSync, 42);
        movementSync.yaw.set(73.0f);
        movementSync.pitch.set(-21.0f);

        for (int tick = 0; tick < 2; tick++) {
            sentPackets.clear();
            task.run();
            List<Object> movementPackets = sentPackets.stream()
                .filter(packet -> packet.getClass().getSimpleName().startsWith("ServerboundMovePlayer"))
                .toList();
            assertEquals(1, movementPackets.size());
            var packet = org.junit.jupiter.api.Assertions.assertInstanceOf(
                ServerboundMovePlayerPosRotPacket.class,
                movementPackets.get(0));
            assertEquals(73.0f, packet.getYaw());
            assertEquals(-21.0f, packet.getPitch());
        }
    }

    @Test
    void plannerJumpHeightsMatchEveryPhysicsTick() throws Exception {
        List<Double> simulatedHeights = new ArrayList<>();
        // With no horizontal displacement, upward box checks and downward support
        // queries expose each simulated tick without adding a production tracing API.
        World world = new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }

            @Override
            public int getBlockAt(Vector3d position) {
                return Math.floor(position.y) == -1 ? STONE : AIR;
            }

            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return false;
            }

            @Override
            public boolean isBoxCollidingExcluding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ, Set<Vector3i> excluded) {
                simulatedHeights.add(minY);
                return false;
            }

            @Override
            public OptionalDouble findHighestCollisionTopExcluding(
                    double minX, double minY, double minZ, double maxX, double maxY,
                    double maxZ, Set<Vector3i> excluded) {
                if (maxY > 1.0e-6) simulatedHeights.add(Math.max(0.0, minY));
                return minY <= 0 && maxY >= 0
                    ? OptionalDouble.of(0) : OptionalDouble.empty();
            }
        };
        Method simulate = StandablePositionResolver.class.getDeclaredMethod(
            "canCompleteJumpTrajectory", World.class, Node.class,
            Node.class, double.class, double.class, double.class);
        simulate.setAccessible(true);
        assertTrue((boolean) simulate.invoke(null, world,
            new Node(0, 0, 0), new Node(0, 0, 0),
            0.0, 0.0, 1.0));
        List<Double> expectedHeights = List.copyOf(simulatedHeights);
        setWorld(world);
        movementSync.position.set(new Vector3d(0.5, 0, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        movementSync.jump();
        for (int tick = 0; tick < expectedHeights.size(); tick++) {
            task.run();
            assertEquals(movementSync.position.get().y, expectedHeights.get(tick), 1.0e-9,
                "planner and physics height at tick " + tick);
        }
        assertEquals(0.42, expectedHeights.get(0), 1.0e-9);
        assertEquals(1.2522, Collections.max(expectedHeights), 1.0e-4);
    }

    @Test
    void jumpUsesImpulseForFirstDisplacementBeforeApplyingGravity() throws Exception {
        setWorld(new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                return Math.floor(position.y) == -1 ? STONE : AIR;
            }
        });
        movementSync.position.set(new Vector3d(0.5, 0.0, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        movementSync.jump();

        task.run();

        assertEquals(0.42, movementSync.position.get().y, 1.0e-9,
            "the jump impulse must be used for the first client tick");
        assertEquals((0.42 - 0.08) * MovementSync.verticalDrag,
            movementSync.velocity.get().y, 1.0e-9,
            "gravity and drag must be retained for the following tick");
    }

    @Test
    void jumpArcClearsAFullBlockBeforePillarPlacementThreshold() throws Exception {
        setWorld(new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                return Math.floor(position.y) == -1 ? STONE : AIR;
            }
        });
        movementSync.position.set(new Vector3d(0.5, 0.0, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        movementSync.jump();
        double maximumY = movementSync.position.get().y;

        for (int tick = 0; tick < 8; tick++) {
            task.run();
            maximumY = Math.max(maximumY, movementSync.position.get().y);
        }

        assertTrue(maximumY >= xin.bbtt.pathfinding.StandablePositionResolver.MAX_JUMP_RISE,
            "planner jump-rise cap must not exceed the real local jump arc: " + maximumY);
    }

    @Test
    void carpetToBlockJumpLandsAfterHittingDestinationCeiling() throws Exception {
        setWorld(new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return AIR;
                if (x == 0 && y == 0) return WHITE_CARPET;
                if (x == 1 && y == 0) return STONE;
                if (x == 1 && y == 3) return STONE;
                return AIR;
            }
        });
        movementSync.position.set(new Vector3d(0.5, 1.0 / 16.0, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        movementSync.jump();
        boolean hitCeiling = false;
        boolean landed = false;

        for (int tick = 0; tick < 20; tick++) {
            movementSync.velocity.updateAndGet(velocity ->
                new Vector3d(MovementSync.movementSpeed, velocity.y, 0));
            task.run();
            Vector3d position = movementSync.position.get();
            if (!movementSync.onGround.get()
                    && Math.abs(movementSync.velocity.get().y) < 1.0e-9
                    && position.y > 1.0) {
                hitCeiling = true;
            }
            if (movementSync.onGround.get()
                    && Math.abs(position.y - 1.0) < 1.0e-9
                    && position.x > 1.0) {
                landed = true;
                break;
            }
        }

        assertTrue(hitCeiling, "the trajectory must exercise upward collision with the ceiling");
        assertTrue(landed, "head contact must still allow landing on the adjacent full block");
    }

    @Test
    void raisedCarpetCatchesThePlayerFootprintBeforeCentering() throws Exception {
        setWorld(new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return AIR;
                if (x == 0 && y == 0) return WHITE_CARPET;
                if (x == 1 && y == 1) return WHITE_CARPET;
                if (x == 1 && y == 3) return STONE;
                return AIR;
            }
        });
        movementSync.position.set(new Vector3d(0.5, 1.0 / 16.0, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        movementSync.jump();
        Vector3d landing = null;

        for (int tick = 0; tick < 20; tick++) {
            movementSync.velocity.updateAndGet(velocity ->
                new Vector3d(MovementSync.movementSpeed, velocity.y, 0));
            task.run();
            Vector3d position = movementSync.position.get();
            if (movementSync.onGround.get()
                    && Math.abs(position.y - 1.0625) < 1.0e-9) {
                landing = new Vector3d(position);
                break;
            }
        }

        assertTrue(landing != null, "the exact downward sweep must land on the raised carpet");
        assertTrue(landing.x + 0.299 > 1.0,
            "the player's footprint must overlap the target support at landing: " + landing);
        assertTrue(landing.x < 1.2,
            "the landing should occur before the center enters the old 0.3 tolerance: " + landing);
    }

    @Test
    void raisedCarpetAlsoCatchesJumpFromBottomSlab() throws Exception {
        setWorld(new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (z != 0) return AIR;
                if (x == 0 && y == 0) return BOTTOM_STONE_SLAB;
                if (x == 1 && y == 1) return WHITE_CARPET;
                if (x == 1 && y == 3) return STONE;
                return AIR;
            }
        });
        movementSync.position.set(new Vector3d(0.5, 0.5, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        movementSync.jump();
        boolean landed = false;

        for (int tick = 0; tick < 20; tick++) {
            movementSync.velocity.updateAndGet(velocity ->
                new Vector3d(MovementSync.movementSpeed, velocity.y, 0));
            task.run();
            Vector3d position = movementSync.position.get();
            if (movementSync.onGround.get()
                    && Math.abs(position.y - 1.0625) < 1.0e-9) {
                landed = true;
                break;
            }
        }

        assertTrue(landed, "the real downward resolver must catch the raised carpet");
    }

    @Test
    void diagonalMotionCannotAccumulateTwoStepUpsInOneTick() throws Exception {
        setWorld(new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (y == -1) return STONE;
                if (x == 1 && y == 0 && z == 0) return BOTTOM_STONE_SLAB;
                if (x == 1 && y == 0 && z == 1) return STONE;
                return AIR;
            }
        });
        movementSync.position.set(new Vector3d(0.5, 0.0, 0.5));
        movementSync.velocity.set(new Vector3d(0.51, 0.0, 0.51));
        movementSync.onGround.set(true);

        task.run();

        Vector3d stepped = movementSync.position.get();
        assertTrue(stepped.x > 0.5, "the first axis should step onto the slab");
        assertTrue(stepped.y <= 0.6 + 1.0e-9,
            "total elevation gain in one tick must not exceed the 0.6-block step limit: " + stepped);
    }

    @Test
    void traversesRaisedOneBlockDoorwayBetweenDirectionalGrindstones() throws Exception {
        setWorld(new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) {
                return true;
            }

            @Override
            public int getBlockAt(Vector3d position) {
                int x = (int) Math.floor(position.x);
                int y = (int) Math.floor(position.y);
                int z = (int) Math.floor(position.z);
                if (y == 0 && z == 0 && x == 2) return DIRT_PATH;
                if (y == 0 && x == 1 && z >= 0 && z <= 3) return SPRUCE_PLANKS;
                if (y == 1 && z == 1 && (x == 0 || x == 2)) return SOUTH_FLOOR_GRINDSTONE;
                if (x == 1 && z == 2 && y == 1) return OPEN_SOUTH_LEFT_SPRUCE_DOOR_LOWER;
                if (x == 1 && z == 2 && y == 2) return OPEN_SOUTH_LEFT_SPRUCE_DOOR_UPPER;
                if ((x == 0 || x == 2) && z >= 2 && z <= 3 && y >= 1 && y <= 2) return STONE;
                return AIR;
            }
        });
        movementSync.position.set(new Vector3d(2.5, 15.0 / 16.0, 0.5));
        movementSync.onGround.set(true);

        movementSync.velocity.set(new Vector3d(-MovementSync.movementSpeed, 0, 0));
        for (int tick = 0; tick < 5; tick++) task.run();
        assertEquals(1.0, movementSync.position.get().y, 1.0e-9);

        movementSync.position.updateAndGet(position -> new Vector3d(1.5, position.y, 0.5));
        movementSync.velocity.set(new Vector3d(0, 0, MovementSync.movementSpeed));
        for (int tick = 0; tick < 15; tick++) task.run();

        Vector3d inside = movementSync.position.get();
        assertEquals(1.5, inside.x, 1.0e-9, "the controller must remain centred in the one-block opening");
        assertTrue(inside.z > 2.7, "the player must clear the open door and reach the interior");
        assertEquals(1.0, inside.y, 1.0e-9);
    }
}
