package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.List;
import java.util.ArrayList;
import xin.bbtt.physics.JumpInputController;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.movements.PathMovement;
import xin.bbtt.pathfinding.BuiltinMovementType;
import xin.bbtt.pathfinding.PathStep;
import java.util.function.BiConsumer;
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
    private static final double PLAYER_HALF_WIDTH = (double) 0.6f / 2;
    private static final double POST_COLLISION_VERTICAL_VELOCITY = -0.08 * 0.9800000190734863D;
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
    private int tickEndPackets;

    @BeforeEach
    void setUp() throws Exception {
        movementSync = new MovementSync();
        movementSync.onRecoveryTeleport();
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
                if (method.getName().equals("isConnected")) return true;
                if (method.getName().equals("send")) {
                    // The per-tick end marker is counted on its own so movement-packet
                    // assertions keep describing movement packets only.
                    if (args[0] instanceof org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.ServerboundClientTickEndPacket) tickEndPackets++;
                    else if (!(args[0] instanceof org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundPlayerInputPacket)) sentPackets.add(args[0]);
                }
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

    @Test
    void powderSnowFallDistanceSurvivesLiveTaskTicks() throws Exception {
        int snow = -1;
        for (int id = 0; id < xin.bbtt.Block.BlockStateParser.getBlockStateRegistrySize(); id++) {
            if (xin.bbtt.Block.BlockStateParser.Instance.parseStateId(id).blockName().equals("powder_snow")) {
                snow = id;
                break;
            }
        }
        assertTrue(snow >= 0);
        final int powderSnow = snow;
        useTerrain(position -> Math.floor(position.x) == 0 && Math.floor(position.y) == -59
                && Math.floor(position.z) == 0 ? powderSnow : AIR);
        movementSync.position.set(new Vector3d(0.5, -55, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(false);
        for (int tick = 0; tick < 30; tick++) {
            task.run();
            if (!movementSync.onGround.get()) continue;
            assertEquals(-59 + (double) 0.9F, movementSync.position.get().y);
            assertEquals(0.0, movementSync.fallDistance.get());
            task.run();
            assertFalse(movementSync.onGround.get());
            assertTrue(movementSync.position.get().y < -59 + (double) 0.9F);
            return;
        }
        throw new AssertionError("Live motion task lost accumulated fall distance before snow impact");
    }

    private boolean collides(Vector3d feetPosition) throws Exception {
        return (boolean) isPlayerBoxColliding.invoke(task, feetPosition);
    }

    @Test
    void groundBodyUsesVanillaFloatWidthWithoutChangingGlidingWidth() throws Exception {
        useTerrain(position -> Math.floor(position.x) == 1 ? STONE : AIR);
        Vector3d vanillaContact = new Vector3d(1 - PLAYER_HALF_WIDTH, 1, 0.5);
        assertFalse(collides(vanillaContact));
        assertTrue(collides(new Vector3d(0.7005, 1, 0.5)),
            "the old narrowed body misses this vanilla AABB overlap");
        assertTrue(collides(new Vector3d(0.7, 1, 0.5)),
            "double 0.3 is still narrower than half of vanilla's float width");
        movementSync.setGliding(true);
        assertFalse(collides(new Vector3d(0.7005, 1, 0.5)),
            "this ground-only change must preserve the gliding body width");
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
    void everyTickEndsWithATickEndPacketEvenWithoutMovement() throws Exception {
        useTerrain(position -> Math.floor(position.y) == 0 ? STONE : AIR);
        assertTick(0.5, 1, 0.5, 0, POST_COLLISION_VERTICAL_VELOCITY, 0, true, false, true);
        int afterFirst = tickEndPackets;
        assertTick(0.5, 1, 0.5, 0, POST_COLLISION_VERTICAL_VELOCITY, 0, true, false, false);
        assertEquals(afterFirst + 1, tickEndPackets);
    }

    @Test
    void stationaryAndHorizontalTicksPreservePacketSuppression() throws Exception {
        useTerrain(position -> Math.floor(position.y) == 0 ? STONE : AIR);
        assertTick(0.5, 1, 0.5, 0, POST_COLLISION_VERTICAL_VELOCITY, 0, true, false, true);
        assertTick(0.5, 1, 0.5, 0, POST_COLLISION_VERTICAL_VELOCITY, 0, true, false, false);
        movementSync.velocity.set(new Vector3d(0.2, 0, 0.1));
        double x = 0.5;
        double z = 0.5;
        double velocityX = 0.2;
        double velocityZ = 0.1;
        for (int tick = 1; tick <= 4; tick++) {
            x += velocityX;
            z += velocityZ;
            velocityX *= 0.6 * 0.91;
            velocityZ *= 0.6 * 0.91;
            assertTick(x, 1, z, velocityX, POST_COLLISION_VERTICAL_VELOCITY, velocityZ, true, false, true);
        }
    }

    @Test
    void fullJumpTraceIncludesVelocityGroundAndPackets() throws Exception {
        useTerrain(position -> Math.floor(position.y) == 0 ? STONE : AIR);
        movementSync.jump();
        double height = 1;
        double speed = (double) 0.42F;
        for (int tick = 0; tick < 12; tick++) {
            height += speed;
            speed = (speed - 0.08) * 0.9800000190734863D;
            if (tick == 11) {
                height = 1;
                speed = POST_COLLISION_VERTICAL_VELOCITY;
            }
            assertTick(0.5, height, 0.5, 0, speed, 0, tick == 11, false, true);
        }
        assertTick(0.5, 1, 0.5, 0, POST_COLLISION_VERTICAL_VELOCITY, 0, true, false, false);
    }

    @Test
    void walkingOffPlatformAppliesGravityForTheFollowingTick() throws Exception {
        useTerrain(position -> Math.floor(position.y) == 0
            || (Math.floor(position.y) == 1 && Math.floor(position.x) == 0) ? STONE : AIR);
        movementSync.position.set(new Vector3d(0.5, 2, 0.5));
        movementSync.velocity.set(new Vector3d(1, 0, 0));
        assertTick(1.5, 2, 0.5, 0.6 * 0.91, POST_COLLISION_VERTICAL_VELOCITY, 0, false, false, true);
        double x = 1.5;
        double velocityX = 0.6 * 0.91;
        double height = 2;
        double speed = POST_COLLISION_VERTICAL_VELOCITY;
        for (int tick = 1; tick <= 5; tick++) {
            height += speed;
            speed = (speed - 0.08) * 0.9800000190734863D;
            if (tick == 5) {
                height = 1;
                speed = POST_COLLISION_VERTICAL_VELOCITY;
            }
            x += velocityX;
            velocityX *= 0.91;
            assertTick(x, height, 0.5, velocityX, speed, 0, tick == 5, false, true);
        }
    }

    @Test
    void supportedEdgeTickReportsGroundContactBeforeFallingOnNextTick() throws Exception {
        useTerrain(position -> Math.floor(position.y) == 0
            && Math.floor(position.x) == 0 ? STONE : AIR);
        movementSync.velocity.set(new Vector3d(1, POST_COLLISION_VERTICAL_VELOCITY, 0));
        double groundDrag = 0.6 * 0.91;
        assertTick(1.5, 1, 0.5, groundDrag, POST_COLLISION_VERTICAL_VELOCITY,
            0, true, false, true);
        assertFalse(movementSync.getWorld().isOnGround(movementSync.position.get()),
            "end-position support must not override this tick's downward collision");
        assertTick(1.5 + groundDrag, 1 + POST_COLLISION_VERTICAL_VELOCITY, 0.5,
            groundDrag * groundDrag,
            (POST_COLLISION_VERTICAL_VELOCITY - 0.08) * 0.9800000190734863D,
            0, false, false, true);
    }

    @Test
    void wallsStopEachAxisAndReportCollisionTransitions() throws Exception {
        for (boolean xAxis : new boolean[]{true, false}) {
            useTerrain(position -> Math.floor(position.y) == 0
                || Math.floor(xAxis ? position.x : position.z) == 1 ? STONE : AIR);
            task = new updateMotionTask();
            movementSync.velocity.set(xAxis ? new Vector3d(0.3, 0, 0) : new Vector3d(0, 0, 0.3));
            // Collision clips travel to the wall face rather than discarding the entire axis.
            double x = xAxis ? 1 - PLAYER_HALF_WIDTH : 0.5;
            double z = xAxis ? 0.5 : 1 - PLAYER_HALF_WIDTH;
            assertTick(x, 1, z, 0, POST_COLLISION_VERTICAL_VELOCITY, 0, true, true, true);
            assertTick(x, 1, z, 0, POST_COLLISION_VERTICAL_VELOCITY, 0, true, false, true);
            assertTick(x, 1, z, 0, POST_COLLISION_VERTICAL_VELOCITY, 0, true, false, false);
        }
    }

    @Test
    void slabStepTraceAppliesGroundDrag() throws Exception {
        useTerrain(position -> Math.floor(position.y) == 0 ? STONE
            : Math.floor(position.y) == 1 && Math.floor(position.x) == 1 ? BOTTOM_STONE_SLAB : AIR);
        movementSync.velocity.set(new Vector3d(0.3, 0, 0));
        double x = 0.5;
        double velocityX = 0.3;
        for (int tick = 1; tick <= 3; tick++) {
            x += velocityX;
            velocityX *= 0.6 * 0.91;
            assertTick(x, 1.5, 0.5, velocityX, POST_COLLISION_VERTICAL_VELOCITY, 0, true, false, true);
        }
    }

    @Test
    void ceilingCancelsJumpWithoutHorizontalCollision() throws Exception {
        useTerrain(position -> Math.floor(position.y) == 0 || Math.floor(position.y) == 3 ? STONE : AIR);
        movementSync.jump();
        // Feet rise until the 1.8-block body touches the ceiling at Y=3.
        assertTick(0.5, 1.2, 0.5, 0, POST_COLLISION_VERTICAL_VELOCITY, 0, false, false, true);
        assertTick(0.5, 1.2 + POST_COLLISION_VERTICAL_VELOCITY, 0.5, 0,
            (POST_COLLISION_VERTICAL_VELOCITY - 0.08) * MovementSync.verticalDrag,
            0, false, false, true);
        assertTick(0.5, 1, 0.5, 0, POST_COLLISION_VERTICAL_VELOCITY, 0, true, false, true);
        assertTick(0.5, 1, 0.5, 0, POST_COLLISION_VERTICAL_VELOCITY, 0, true, false, false);
    }

    @Test
    void unloadedChunkStillMovesAndSendsPositionAndTickEnd() throws Exception {
        setWorld(new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return false; }
        });
        movementSync.position.set(new Vector3d(0.5, 4, 0.5));
        movementSync.velocity.set(new Vector3d(0.2, -0.3, 0.4));
        movementSync.onGround.set(true);
        double falling = -0.1 * MovementSync.verticalDrag;
        assertTick(0.7, 3.7, 0.9, 0.2 * 0.546, falling, 0.4 * 0.546, false, false, true);
        assertTick(0.7 + 0.2 * 0.546, 3.7 + falling, 0.9 + 0.4 * 0.546,
                0.2 * 0.546 * 0.91, falling, 0.4 * 0.546 * 0.91, false, false, true);
        assertEquals(2, tickEndPackets);
    }

    @Test
    void unloadedChunkBelowWorldStopsVerticalAcceleration() throws Exception {
        setWorld(new World());
        movementSync.position.set(new Vector3d(0.5, World.getMinWorldY() - 1, 0.5));
        movementSync.velocity.set(new Vector3d(0, -0.3, 0));
        movementSync.onGround.set(false);
        assertTick(0.5, World.getMinWorldY() - 1.3, 0.5, 0, 0, 0, false, false, true);
        assertEquals(1, tickEndPackets);
    }

    @Test
    void chunkArrivalResumesGravityWithoutResettingMomentum() throws Exception {
        boolean[] loaded = {false};
        setWorld(new World() {
            @Override public boolean chunkLoaded(int chunkX, int chunkZ) { return loaded[0]; }
        });
        movementSync.position.set(new Vector3d(0.5, 4, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(false);
        double falling = -0.1 * MovementSync.verticalDrag;
        assertTick(0.5, 4, 0.5, 0, falling, 0, false, false, true);
        loaded[0] = true;
        assertTick(0.5, 4 + falling, 0.5, 0, (falling - 0.08) * MovementSync.verticalDrag,
                0, false, false, true);
        assertEquals(2, tickEndPackets);
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
    void plannerJumpTrajectoryMatchesEveryPhysicsTick() throws Exception {
        Node source = new Node(0, 1, 0);
        for (Node target : new Node[] {new Node(1, 2, 0), new Node(2, 1, 0),
                new Node(3, 1, 0), new Node(2, 1, 2)}) {
            boolean gapJump = target.y == source.y;
            World world = new World() {
                @Override
                public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }

                @Override
                public int getBlockAt(Vector3d position) {
                    int x = (int) Math.floor(position.x);
                    int y = (int) Math.floor(position.y);
                    int z = (int) Math.floor(position.z);
                    return (x == source.x && y == source.y - 1 && z == source.z)
                            || (x == target.x && y == target.y - 1 && z == target.z)
                            ? STONE : AIR;
                }
            };
            assertTrue(gapJump ? StandablePositionResolver.canGapJump(world, source, target)
                    : StandablePositionResolver.canJump(world, source, target), target.toString());
            setWorld(world);
            task = new updateMotionTask();
            movementSync.clearInput();
            movementSync.position.set(new Vector3d(0.5, 1, 0.5));
            movementSync.velocity.set(new Vector3d());
            movementSync.onGround.set(true);
            List<JumpInputController.Command> previewCommands = new ArrayList<>();
            List<PhysicsResult> previewResults = new ArrayList<>();
            Method preview = StandablePositionResolver.class.getDeclaredMethod(
                    "canJump", World.class, Node.class, Node.class, boolean.class, BiConsumer.class);
            preview.setAccessible(true);
            BiConsumer<JumpInputController.Command, PhysicsResult> observer = (command, result) -> {
                previewCommands.add(command);
                previewResults.add(result);
            };
            assertEquals(true, preview.invoke(null, world, source, target, gapJump, observer));
            assertFalse(previewResults.isEmpty());
            PathMovement movement = new PathMovement(List.of(
                    new PathStep(source, BuiltinMovementType.WALK),
                    new PathStep(target, gapJump ? BuiltinMovementType.GAP_JUMP : BuiltinMovementType.JUMP)));
            movement.init();
            double maximumRise = 0;
            boolean landed = false;
            for (int tick = 0; tick < previewResults.size(); tick++) {
                PhysicsResult expected = previewResults.get(tick);
                String message = target + " at tick " + tick;
                movement.onTick();
                assertFalse(movement.isFinished(), message + " must not finish before landing");
                assertEquals(previewCommands.get(tick).input(), movementSync.getInput(), message + " keys");
                assertEquals(0.0, Math.IEEEremainder(movementSync.yaw.get() - previewCommands.get(tick).yaw(), 360.0),
                        1.0e-6, message + " yaw direction");
                task.run();
                assertEquals(expected.position().x, movementSync.position.get().x, 1.0e-9, message + " x");
                assertEquals(expected.position().y, movementSync.position.get().y, 1.0e-9, message + " y");
                assertEquals(expected.position().z, movementSync.position.get().z, 1.0e-9, message + " z");
                assertEquals(expected.velocity().x, movementSync.velocity.get().x, 1.0e-9, message + " velocity x");
                assertEquals(expected.velocity().y, movementSync.velocity.get().y, 1.0e-9, message + " velocity y");
                assertEquals(expected.velocity().z, movementSync.velocity.get().z, 1.0e-9, message + " velocity z");
                assertEquals(expected.onGround(), movementSync.onGround.get(), message + " grounded");
                if (tick == 0) assertEquals((double) 0.42F, expected.position().y - source.y, 1.0e-9);
                maximumRise = Math.max(maximumRise, expected.position().y - source.y);
                assertEquals(tick == previewResults.size() - 1, expected.onGround(),
                        message + " preview must run through landing");
                if (!expected.onGround()) continue;
                Vector3d position = expected.position();
                assertEquals(target.y, position.y, 1.0e-9, message + " landing height");
                assertTrue(position.x + PLAYER_HALF_WIDTH > target.x && position.x - PLAYER_HALF_WIDTH < target.x + 1
                        && position.z + PLAYER_HALF_WIDTH > target.z && position.z - PLAYER_HALF_WIDTH < target.z + 1,
                        message + " must land on the planned target");
                landed = true;
                break;
            }
            assertTrue(landed, "must finish the planned jump: " + target);
            assertEquals(1.2522, maximumRise, 1.0e-4, "jump apex: " + target);
        }
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

        assertEquals((double) 0.42F, movementSync.position.get().y, 1.0e-9,
            "the jump impulse must be used for the first client tick");
        assertEquals(((double) 0.42F - 0.08) * MovementSync.verticalDrag,
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
        // Exercise real acceleration from rest, not a terminal-speed override each tick.
        movementSync.yaw.set(-90.0f);
        movementSync.setInput(new PlayerInput(true, false, false, false, false, false, false));
        movementSync.jump();
        boolean hitCeiling = false;
        boolean landed = false;

        for (int tick = 0; tick < 20; tick++) {
            task.run();
            Vector3d position = movementSync.position.get();
            if (!movementSync.onGround.get()
                    && Math.abs(movementSync.velocity.get().y - POST_COLLISION_VERTICAL_VELOCITY) < 1.0e-9
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
        // Exercise real acceleration from rest, not a terminal-speed override each tick.
        movementSync.yaw.set(-90.0f);
        movementSync.setInput(new PlayerInput(true, false, false, false, false, false, false));
        movementSync.jump();
        Vector3d landing = null;

        for (int tick = 0; tick < 20; tick++) {
            task.run();
            Vector3d position = movementSync.position.get();
            if (movementSync.onGround.get()
                    && Math.abs(position.y - 1.0625) < 1.0e-9) {
                landing = new Vector3d(position);
                break;
            }
        }

        assertTrue(landing != null, "the exact downward sweep must land on the raised carpet");
        assertTrue(landing.x + PLAYER_HALF_WIDTH > 1.0,
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

        for (int tick = 0; tick < 5; tick++) {
            movementSync.velocity.set(new Vector3d(-MovementSync.movementSpeed, 0, 0));
            task.run();
        }
        assertEquals(1.0, movementSync.position.get().y, 1.0e-9);

        movementSync.position.updateAndGet(position -> new Vector3d(1.5, position.y, 0.5));
        for (int tick = 0; tick < 15; tick++) {
            movementSync.velocity.set(new Vector3d(0, 0, MovementSync.movementSpeed));
            task.run();
        }

        Vector3d inside = movementSync.position.get();
        assertEquals(1.5, inside.x, 1.0e-9, "the controller must remain centred in the one-block opening");
        assertTrue(inside.z > 2.7, "the player must clear the open door and reach the interior");
        assertEquals(1.0, inside.y, 1.0e-9);
    }
}
