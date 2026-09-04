package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
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
    private static final int BOTTOM_STONE_SLAB = 13197;
    private static final int OPEN_NORTH_LEFT_JUNGLE_DOOR = 13995;
    private static final int OPEN_SOUTH_LEFT_SPRUCE_DOOR_LOWER = 13883;
    private static final int OPEN_SOUTH_LEFT_SPRUCE_DOOR_UPPER = 13875;
    private static final int SOUTH_FLOOR_GRINDSTONE = 20571;
    private MovementSync movementSync;
    private updateMotionTask task;
    private Method isPlayerBoxColliding;

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
