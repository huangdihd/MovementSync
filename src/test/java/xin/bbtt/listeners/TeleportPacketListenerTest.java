package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.ClientSession;
import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PositionElement;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundPlayerPositionPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundAcceptTeleportationPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;

/** Teleports are resolved like the vanilla client's EntityPosition.apply. */
class TeleportPacketListenerTest {
    private final MovementSync movementSync = new MovementSync();
    private final List<Object> packets = new ArrayList<>();
    private Object previousSession;
    private Session session;

    @BeforeEach
    void installSession() throws Exception {
        MovementSync.INSTANCE = movementSync;
        Field sessionField = Bot.class.getDeclaredField("session");
        sessionField.setAccessible(true);
        previousSession = sessionField.get(Bot.INSTANCE);
        session = (Session) Proxy.newProxyInstance(getClass().getClassLoader(),
            new Class<?>[]{ClientSession.class}, (proxy, method, arguments) -> {
                if (method.getName().equals("isConnected")) return true;
                if (method.getName().equals("send")) packets.add(arguments[0]);
                return null;
            });
        sessionField.set(Bot.INSTANCE, session);
        movementSync.position.set(new Vector3d(10, 70, -5));
        movementSync.velocity.set(new Vector3d(0.3, -0.2, 1.1));
        movementSync.yaw.set(90f);
        movementSync.pitch.set(20f);
    }

    @AfterEach
    void restore() throws Exception {
        Field sessionField = Bot.class.getDeclaredField("session");
        sessionField.setAccessible(true);
        sessionField.set(Bot.INSTANCE, previousSession);
        MovementSync.INSTANCE = null;
    }

    @Test
    void absoluteTeleportTakesPositionRotationAndVelocityFromThePacket() {
        new TeleportPacketListener().packetReceived(session,
            new ClientboundPlayerPositionPacket(7, 1.5, 64, 2.5, 0, 0, 0, 180f, -10f));

        assertVector(1.5, 64, 2.5, movementSync.position.get());
        assertVector(0, 0, 0, movementSync.velocity.get());
        assertEquals(180f, movementSync.yaw.get());
        assertEquals(-10f, movementSync.pitch.get());
        assertEquals(7, assertInstanceOf(ServerboundAcceptTeleportationPacket.class, packets.get(0)).getId());
        ServerboundMovePlayerPosRotPacket echo =
            assertInstanceOf(ServerboundMovePlayerPosRotPacket.class, packets.get(1));
        assertEquals(1.5, echo.getX());
        assertEquals(64, echo.getY());
        assertEquals(2.5, echo.getZ());
        assertEquals(180f, echo.getYaw());
        assertEquals(-10f, echo.getPitch());
    }

    @Test
    void relativeFlagsAddToTheCurrentStateAndKeepVelocity() {
        new TeleportPacketListener().packetReceived(session,
            new ClientboundPlayerPositionPacket(8, 1, 2, 3, 0.1, 0, 0, 5f, 80f,
                PositionElement.X, PositionElement.Y, PositionElement.Z,
                PositionElement.Y_ROT, PositionElement.X_ROT,
                PositionElement.DELTA_X, PositionElement.DELTA_Y, PositionElement.DELTA_Z));

        assertVector(11, 72, -2, movementSync.position.get());
        assertVector(0.4, -0.2, 1.1, movementSync.velocity.get());
        assertEquals(95f, movementSync.yaw.get());
        assertEquals(90f, movementSync.pitch.get(), "pitch is clamped to straight down");
    }

    @Test
    void rotateDeltaTurnsTheKeptVelocityWithTheYawChange() {
        movementSync.velocity.set(new Vector3d(0, 0, 1));
        movementSync.pitch.set(0f);
        // Yaw goes from 90 to 0: the kept velocity is rotated by +90 degrees about Y.
        new TeleportPacketListener().packetReceived(session,
            new ClientboundPlayerPositionPacket(9, 0, 64, 0, 0, 0, 0, 0f, 0f,
                PositionElement.ROTATE_DELTA,
                PositionElement.DELTA_X, PositionElement.DELTA_Y, PositionElement.DELTA_Z));

        assertVector(1, 0, 0, movementSync.velocity.get());
    }

    private static void assertVector(double x, double y, double z, Vector3d actual) {
        assertEquals(x, actual.x, 1.0e-9);
        assertEquals(y, actual.y, 1.0e-9);
        assertEquals(z, actual.z, 1.0e-9);
    }
}
