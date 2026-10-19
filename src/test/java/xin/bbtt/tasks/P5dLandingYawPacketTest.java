package xin.bbtt.tasks;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;
import org.geysermc.mcprotocollib.network.ClientSession;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerRotPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;
import xin.bbtt.movements.GroundedFlightStop;
import xin.bbtt.world.World;

final class P5dLandingYawPacketTest {
    private final List<Object> packets = new ArrayList<>();
    private MovementSync sync;
    private updateMotionTask task;

    @BeforeEach
    void setUp() throws Exception {
        sync = new MovementSync();
        sync.onRecoveryTeleport();
        Field world = MovementSync.class.getDeclaredField("world");
        world.setAccessible(true);
        world.set(sync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public int getBlockAt(Vector3d p) { return Math.floor(p.y) == 0 ? 1 : 0; }
        });
        sync.position.set(new Vector3d(0.5, 1, 0.5));
        sync.onGround.set(true);
        sync.setGliding(true);
        setBotField("running", true);
        Bot.INSTANCE.setServer(Server.Game);
        setBotField("session", Proxy.newProxyInstance(ClientSession.class.getClassLoader(),
                new Class<?>[]{ClientSession.class}, (proxy, method, arguments) -> {
                    if (method.getName().equals("send")) packets.add(arguments[0]);
                    return method.getReturnType() == boolean.class ? true : null;
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

    @ParameterizedTest
    @ValueSource(floats = {179f, -179f, 539f, -539f})
    void groundedBrakingKeepsSuccessivePacketYawContinuous(float initialYaw) {
        sync.yaw.set(initialYaw);
        task.run();
        float previous = emittedYaw();
        // Cross both signed boundaries, including accumulated full turns.
        float firstDirection = initialYaw < 0 ? 178f : -178f;
        for (float direction : new float[]{firstDirection, -firstDirection, -170f, 170f}) {
            packets.clear();
            double radians = Math.toRadians(direction);
            sync.velocity.set(new Vector3d(Math.sin(radians) * 0.05, 0, -Math.cos(radians) * 0.05));
            GroundedFlightStop.steerGroundedGlide(sync);
            sync.queueBeforeMovement(new ServerboundUseItemPacket(Hand.MAIN_HAND, 42, 0, 0));
            task.run();
            float emitted = emittedYaw();
            assertTrue(Math.abs(emitted - previous) <= 180f,
                    "successive emitted yaw jumped from " + previous + " to " + emitted);
            assertEquals(0, Math.IEEEremainder(emitted - direction, 360), 0.0001,
                    "continuity must preserve the braking direction");
            assertEquals(sync.getLastFlightTick().yaw(), emitted, "physics and movement packet rotation");
            var use = packets.stream().filter(ServerboundUseItemPacket.class::isInstance)
                    .map(ServerboundUseItemPacket.class::cast).findFirst().orElseThrow();
            assertEquals(emitted, use.getYRot(), "use-item and movement packet rotation");
            assertEquals(sync.getLastFlightTick().pitch(), use.getXRot());
            previous = emitted;
        }
    }

    private float emittedYaw() {
        for (Object packet : packets) {
            if (packet instanceof ServerboundMovePlayerPosRotPacket p) return p.getYaw();
            if (packet instanceof ServerboundMovePlayerRotPacket p) return p.getYaw();
        }
        fail("actual motion task emitted no rotation packet");
        return Float.NaN;
    }

    private static void setBotField(String name, Object value) throws Exception {
        Field field = Bot.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(Bot.INSTANCE, value);
    }
}
