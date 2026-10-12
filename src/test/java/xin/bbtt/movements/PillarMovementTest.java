package xin.bbtt.movements;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;
import org.cloudburstmc.math.vector.Vector3i;
import org.geysermc.mcprotocollib.network.ClientSession;
import org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSwingPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemOnPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

final class PillarMovementTest {
    private static final double PLAYER_HALF_WIDTH = (double) 0.6f / 2;
    @AfterEach
    void reset() throws Exception {
        setBotField("session", null);
        MovementSync.INSTANCE = null;
    }

    @Test
    void jumpsBeforePlacingBlockUnderAirborneFeet() throws Exception {
        List<Object> packets = new ArrayList<>();
        setBotField("session", Proxy.newProxyInstance(
            ClientSession.class.getClassLoader(),
            new Class<?>[]{ClientSession.class},
            (proxy, method, args) -> {
                if (method.getName().equals("send") && args != null && args.length > 0) packets.add(args[0]);
                Class<?> type = method.getReturnType();
                if (!type.isPrimitive()) return null;
                if (type == boolean.class) return false;
                if (type == int.class) return 0;
                if (type == long.class) return 0L;
                if (type == float.class) return 0.0f;
                if (type == double.class) return 0.0d;
                if (type == byte.class) return (byte) 0;
                if (type == short.class) return (short) 0;
                if (type == char.class) return '\0';
                return null;
            }
        ));
        MovementSync movementSync = new MovementSync();
        movementSync.position.set(new Vector3d(0.5, 64.0, 0.5));
        movementSync.velocity.set(new Vector3d());
        movementSync.onGround.set(true);
        PillarMovement movement = new PillarMovement(
            Vector3i.from(0, 64, 0),
            Vector3i.from(0, 63, 0),
            Direction.UP,
            -1
        );

        movement.init();
        movement.onTick();

        assertTrue(movementSync.onGround.get());
        assertEquals(new Vector3d(), movementSync.velocity.get());
        PlayerInput input = movementSync.snapshotInput();
        assertTrue(input.jump());
        assertFalse(movementSync.snapshotInput().jump(), "jump must be a one-shot request");
        assertTrue(packets.isEmpty(), "placement must wait until the feet leave the target cell");
        assertFalse(movement.isFinished());

        World world = new World() {
            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return minY < 64.0;
            }
        };
        for (int tick = 0; tick < 3; tick++) {
            PhysicsResult result = new WalkingPhysics(input, movementSync.yaw.get()).step(
                    world, new PhysicsState(movementSync.position.get(), movementSync.velocity.get(),
                            movementSync.onGround.get()), PLAYER_HALF_WIDTH, 1.8);
            movementSync.position.set(result.position());
            movementSync.velocity.set(result.velocity());
            movementSync.onGround.set(result.onGround());
            assertFalse(movementSync.onGround.get());
            if (tick == 0) {
                assertEquals(64.42, result.position().y, 1.0e-9);
                assertEquals((0.42 - 0.08) * 0.9800000190734863D, result.velocity().y, 1.0e-9);
            }
            movement.onTick();
            if (tick < 2) {
                assertTrue(result.position().y < 65.0);
                assertTrue(packets.isEmpty(), "placement must wait until the feet leave the target cell");
                assertFalse(movement.isFinished());
            }
            input = movementSync.snapshotInput();
            assertFalse(input.jump(), "waiting for clearance must not request another jump");
        }
        assertTrue(movementSync.position.get().y >= 65.0);

        assertEquals(2, packets.size());
        assertTrue(packets.get(0) instanceof ServerboundUseItemOnPacket);
        assertTrue(packets.get(1) instanceof ServerboundSwingPacket);
        assertTrue(movement.isFinished());
    }

    private static void setBotField(String name, Object value) throws Exception {
        Field field = Bot.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(Bot.INSTANCE, value);
    }
}
