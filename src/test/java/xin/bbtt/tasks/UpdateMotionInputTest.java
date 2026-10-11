package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;
import org.geysermc.mcprotocollib.network.ClientSession;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerState;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.ServerboundClientTickEndPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundPlayerInputPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerCommandPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.world.World;

final class UpdateMotionInputTest {
    private final List<Object> packets = new ArrayList<>();
    private MovementSync movementSync;
    private updateMotionTask task;
    private Runnable duringCollision = () -> {};
    private Runnable duringInputSend = () -> {};

    @BeforeEach
    void setUp() throws Exception {
        movementSync = new MovementSync();
        movementSync.entityId = 7;
        Field world = MovementSync.class.getDeclaredField("world");
        world.setAccessible(true);
        world.set(movementSync, new World() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }

            @Override
            public int getBlockAt(Vector3d position) {
                return Math.floor(position.y) == 0 ? 1 : 0;
            }

            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                duringCollision.run();
                return super.isBoxColliding(minX, minY, minZ, maxX, maxY, maxZ);
            }
        });
        resetToGround();
        setBotField("running", true);
        Bot.INSTANCE.setServer(Server.Game);
        setBotField("session", Proxy.newProxyInstance(ClientSession.class.getClassLoader(),
                new Class<?>[]{ClientSession.class}, (proxy, method, arguments) -> {
                    if (method.getName().equals("send")) {
                        packets.add(arguments[0]);
                        if (arguments[0] instanceof ServerboundPlayerInputPacket) duringInputSend.run();
                    }
                    if (method.getReturnType() == boolean.class) return true;
                    return null;
                }));
        task = new updateMotionTask();
    }

    @AfterEach
    void tearDown() throws Exception {
        setBotField("running", false);
        setBotField("session", null);
        Bot.INSTANCE.setServer(null);
        MovementSync.INSTANCE = null;
    }

    @Test
    void inputSprintAndQueuedActionsPrecedeMovementAndTickEnd() {
        movementSync.setInput(new PlayerInput(true, false, false, false, false, false, true));
        var first = new ServerboundPlayerCommandPacket(7, PlayerState.START_ELYTRA_FLYING);
        var second = new ServerboundPlayerCommandPacket(7, PlayerState.START_ELYTRA_FLYING);
        movementSync.queueBeforeMovement(first);
        movementSync.queueBeforeMovement(second);

        task.run();

        assertEquals(6, packets.size());
        assertInstanceOf(ServerboundPlayerInputPacket.class, packets.get(0));
        assertEquals(PlayerState.START_SPRINTING,
                assertInstanceOf(ServerboundPlayerCommandPacket.class, packets.get(1)).getState());
        assertSame(first, packets.get(2));
        assertSame(second, packets.get(3));
        assertInstanceOf(ServerboundMovePlayerPosPacket.class, packets.get(4));
        assertInstanceOf(ServerboundClientTickEndPacket.class, packets.get(5));
        assertNull(movementSync.pollBeforeMovementPacket());

        packets.clear();
        movementSync.setInput(new PlayerInput(true, false, false, false, false, false, true));
        task.run();
        assertEquals(2, packets.size(), "equal held input sends neither input nor sprint again");
        assertInstanceOf(ServerboundMovePlayerPosPacket.class, packets.get(0));
        assertInstanceOf(ServerboundClientTickEndPacket.class, packets.get(1));

        packets.clear();
        movementSync.clearInput();
        task.run();
        assertInstanceOf(ServerboundPlayerInputPacket.class, packets.get(0));
        assertEquals(PlayerState.STOP_SPRINTING,
                assertInstanceOf(ServerboundPlayerCommandPacket.class, packets.get(1)).getState());
        assertInstanceOf(ServerboundClientTickEndPacket.class, packets.get(packets.size() - 1));
    }

    @Test
    void groundStateOnlyChangeSendsMovementBeforeTickEndThenSuppressesDuplicates() {
        movementSync.onGround.set(false);
        task.run();
        assertEquals(2, packets.size());
        var airborne = assertInstanceOf(ServerboundMovePlayerPosPacket.class, packets.get(0));
        assertFalse(airborne.isOnGround());
        assertFalse(airborne.isHorizontalCollision());
        assertInstanceOf(ServerboundClientTickEndPacket.class, packets.get(1));

        packets.clear();
        task.run();
        assertEquals(2, packets.size(), "ground contact alone must send movement");
        var grounded = assertInstanceOf(ServerboundMovePlayerPosPacket.class, packets.get(0));
        assertEquals(airborne.getX(), grounded.getX());
        assertEquals(airborne.getY(), grounded.getY());
        assertEquals(airborne.getZ(), grounded.getZ());
        assertTrue(grounded.isOnGround());
        assertTrue(movementSync.onGround.get());
        assertFalse(grounded.isHorizontalCollision());
        assertInstanceOf(ServerboundClientTickEndPacket.class, packets.get(1));

        packets.clear();
        task.run();
        assertEquals(1, packets.size());
        assertInstanceOf(ServerboundClientTickEndPacket.class, packets.get(0));
    }

    @Test
    void jumpRequestDoesNotMutateVelocityAndIsConsumedOnce() {
        Vector3d initial = movementSync.velocity.get();
        movementSync.jump();
        assertSame(initial, movementSync.velocity.get());
        assertEquals(0, initial.y);
        task.run();
        assertEquals(1.42, movementSync.position.get().y, 1.0e-9);
        assertEquals(PlayerInput.NONE, movementSync.getInput());
        resetToGround();
        task.run();
        assertEquals(1, movementSync.position.get().y, 1.0e-9);
        assertEquals(List.of(
                new ServerboundPlayerInputPacket(false, false, false, false, true, false, false),
                new ServerboundPlayerInputPacket(false, false, false, false, false, false, false)),
                packets.stream().filter(ServerboundPlayerInputPacket.class::isInstance).toList());
    }

    @Test
    void heldJumpRepeatsAfterTenTicksAndReleaseResetsCooldown() {
        movementSync.setInput(PlayerInput.NONE.withJump(true));
        task.run();
        assertEquals(1.42, movementSync.position.get().y, 1.0e-9);
        // Simulate early landings to isolate cooldown from the twelve-tick jump arc.
        for (int tick = 1; tick < 10; tick++) {
            resetToGround();
            task.run();
            assertEquals(1, movementSync.position.get().y, 1.0e-9, "cooldown tick " + tick);
        }
        resetToGround();
        task.run();
        assertEquals(1.42, movementSync.position.get().y, 1.0e-9);
        movementSync.clearInput();
        resetToGround();
        task.run();
        movementSync.setInput(PlayerInput.NONE.withJump(true));
        task.run();
        assertEquals(1.42, movementSync.position.get().y, 1.0e-9);
    }

    @Test
    void inputReplacementDuringSendAppliesOnlyOnNextTick() {
        movementSync.setInput(PlayerInput.NONE.withJump(true));
        duringInputSend = movementSync::clearInput;
        task.run();
        assertEquals(1.42, movementSync.position.get().y, 1.0e-9,
                "physics must use the same snapshot as the emitted packet");
        resetToGround();
        task.run();
        assertEquals(1, movementSync.position.get().y, 1.0e-9);
    }

    @Test
    void concurrentVelocityReplacementWinsForEveryComponentEvenWhenEqual() {
        for (Vector3d replacement : List.of(new Vector3d(0.2, 0, 0.1), new Vector3d(3, 4, 5))) {
            resetToGround();
            movementSync.velocity.set(new Vector3d(0.2, 0, 0.1));
            duringCollision = () -> movementSync.velocity.set(replacement);
            task.run();
            assertSame(replacement, movementSync.velocity.get());
            assertEquals(new Vector3d(0.7, 1, 0.6), movementSync.position.get(),
                    "the in-flight tick still integrates its original velocity snapshot");
        }
    }

    @Test
    void clearInputDiscardsHeldKeysAndPendingOneShotJump() {
        movementSync.setInput(new PlayerInput(true, true, true, true, true, true, true));
        movementSync.jump();
        movementSync.clearInput();
        assertEquals(PlayerInput.NONE, movementSync.getInput());
        assertEquals(PlayerInput.NONE, movementSync.snapshotInput());
        assertEquals(PlayerInput.NONE, movementSync.snapshotInput());
    }

    private void resetToGround() {
        movementSync.position.set(new Vector3d(0.5, 1, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
    }

    private static void setBotField(String name, Object value) throws Exception {
        Field field = Bot.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(Bot.INSTANCE, value);
    }
}
