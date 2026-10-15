package xin.bbtt.tasks;

import org.geysermc.mcprotocollib.network.ClientSession;
import org.geysermc.mcprotocollib.protocol.data.game.WeightedList;
import org.geysermc.mcprotocollib.protocol.data.game.level.particle.Particle;
import org.geysermc.mcprotocollib.protocol.data.game.level.particle.ParticleType;
import org.geysermc.mcprotocollib.protocol.data.game.level.sound.BuiltinSound;
import org.geysermc.mcprotocollib.protocol.packet.common.clientbound.ClientboundPingPacket;
import org.geysermc.mcprotocollib.protocol.packet.common.serverbound.ServerboundPongPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundSetEntityMotionPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundExplodePacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.ServerboundClientTickEndPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import xin.bbtt.MovementSync;
import xin.bbtt.listeners.PingPacketListener;
import xin.bbtt.listeners.VelocityPacketListener;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;
import xin.bbtt.world.World;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.*;

class UpdateMotionKnockbackTest {
    private final List<Object> packets = new ArrayList<>();
    private MovementSync movementSync;
    private MovementSync previous;
    private ClientSession session;
    private Object previousSession;
    private Object previousRunning;
    private Server previousServer;
    private Runnable duringPhysics;

    @BeforeEach
    void install() throws Exception {
        previous = MovementSync.INSTANCE;
        previousSession = botField("session").get(Bot.INSTANCE);
        previousRunning = botField("running").get(Bot.INSTANCE);
        previousServer = Bot.INSTANCE.getServer();
        movementSync = new MovementSync();
        movementSync.entityId = 42;
        movementSync.onGround.set(false);
        movementSync.position.set(new Vector3d(0.5, 20, 0.5));
        movementSync.velocity.set(new Vector3d(0.2, 0.1, 0.3));
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) {
                if (duringPhysics != null) {
                    Runnable callback = duringPhysics;
                    duringPhysics = null;
                    callback.run();
                }
                return true;
            }
            @Override public int getBlockAt(Vector3d position) {
                chunkLoaded(0, 0);
                return 0;
            }
            @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                                                    double maxX, double maxY, double maxZ) {
                chunkLoaded(0, 0);
                return false;
            }
        };
        Field worldField = MovementSync.class.getDeclaredField("world");
        worldField.setAccessible(true);
        worldField.set(movementSync, world);
        session = (ClientSession) Proxy.newProxyInstance(ClientSession.class.getClassLoader(),
                new Class<?>[]{ClientSession.class}, (proxy, method, arguments) -> {
                    if (method.getName().equals("send")) packets.add(arguments[0]);
                    if (method.getReturnType() == boolean.class) return true;
                    return null;
                });
        botField("session").set(Bot.INSTANCE, session);
        botField("running").set(Bot.INSTANCE, true);
        Bot.INSTANCE.setServer(Server.Game);
    }

    @AfterEach
    void restore() throws Exception {
        botField("session").set(Bot.INSTANCE, previousSession);
        botField("running").set(Bot.INSTANCE, previousRunning);
        Bot.INSTANCE.setServer(previousServer);
        MovementSync.INSTANCE = previous;
    }

    @ParameterizedTest
    @CsvSource({"false,false", "false,true", "true,false", "true,true"})
    void impulseBeforeTickIsConsumedByThatPhysicsStep(boolean gliding, boolean explosion) {
        movementSync.setGliding(gliding);
        var task = new updateMotionTask();
        receiveImpulse(explosion);
        Vector3d impulseVelocity = new Vector3d(movementSync.velocity.get());
        new PingPacketListener().packetReceived(session, new ClientboundPingPacket(17));
        task.run();
        assertInstanceOf(ServerboundPongPacket.class, packets.get(0));
        assertInstanceOf(ServerboundClientTickEndPacket.class, packets.get(packets.size() - 1));
        assertTrue(movementSync.position.get().x > 1, "impulse must affect this tick, not the next");
        assertNotEquals(impulseVelocity, movementSync.velocity.get(), "physics must apply drag");
    }

    @ParameterizedTest
    @CsvSource({"false,false", "false,true", "true,false", "true,true"})
    void packetArrivingDuringPhysicsSurvivesWritebackAndPrecedesPong(boolean gliding, boolean explosion)
            throws Exception {
        movementSync.setGliding(gliding);
        var task = new updateMotionTask();
        task.run();
        Vector3d expectedPhysicsVelocity = new Vector3d(movementSync.velocity.get());
        movementSync.position.set(new Vector3d(0.5, 20, 0.5));
        movementSync.velocity.set(new Vector3d(0.2, 0.1, 0.3));
        packets.clear();
        AtomicReference<Throwable> failure = new AtomicReference<>();
        Thread receiver = new Thread(() -> {
            try {
                receiveImpulse(explosion);
                new PingPacketListener().packetReceived(session, new ClientboundPingPacket(23));
            } catch (Throwable error) {
                failure.set(error);
            }
        });
        duringPhysics = () -> {
            assertTrue(Thread.holdsLock(movementSync));
            receiver.start();
            long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(3);
            while (receiver.getState() != Thread.State.BLOCKED && System.nanoTime() < deadline) {
                Thread.yield();
            }
            assertEquals(Thread.State.BLOCKED, receiver.getState(), "packet must wait for the tick boundary");
        };
        task.run();
        receiver.join(3000);
        assertFalse(receiver.isAlive());
        assertNull(failure.get());
        assertNull(duringPhysics, "test must inject while physics is executing");
        Vector3d expected = new Vector3d(0.8, 0.4, -0.2);
        if (explosion) expected.add(expectedPhysicsVelocity);
        assertEquals(expected, movementSync.velocity.get());
        assertInstanceOf(ServerboundClientTickEndPacket.class, packets.get(packets.size() - 2));
        assertEquals(23, assertInstanceOf(ServerboundPongPacket.class, packets.get(packets.size() - 1)).getId());
        Vector3d beforeNextTick = new Vector3d(movementSync.position.get());
        task.run();
        assertTrue(movementSync.position.get().x > beforeNextTick.x + 0.5);
    }

    private void receiveImpulse(boolean explosion) {
        var impulse = org.cloudburstmc.math.vector.Vector3d.from(0.8, 0.4, -0.2);
        var listener = new VelocityPacketListener();
        if (!explosion) {
            listener.packetReceived(session, new ClientboundSetEntityMotionPacket(42, impulse));
            return;
        }
        listener.packetReceived(session, new ClientboundExplodePacket(
                org.cloudburstmc.math.vector.Vector3d.ZERO, 4, 0, impulse,
                new Particle(ParticleType.EXPLOSION, null), BuiltinSound.ENTITY_GENERIC_EXPLODE,
                new WeightedList<>(List.of())));
    }

    private static Field botField(String name) throws Exception {
        Field field = Bot.class.getDeclaredField(name);
        field.setAccessible(true);
        return field;
    }
}
