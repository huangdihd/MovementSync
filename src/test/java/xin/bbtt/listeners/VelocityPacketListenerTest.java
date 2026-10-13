package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.protocol.data.game.WeightedList;
import org.geysermc.mcprotocollib.protocol.data.game.level.particle.Particle;
import org.geysermc.mcprotocollib.protocol.data.game.level.particle.ParticleType;
import org.geysermc.mcprotocollib.protocol.data.game.level.sound.BuiltinSound;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.ClientboundSetEntityMotionPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundExplodePacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class VelocityPacketListenerTest {
    private final MovementSync movementSync = new MovementSync();
    private final VelocityPacketListener listener = new VelocityPacketListener();
    private MovementSync previous;

    @BeforeEach
    void install() {
        previous = MovementSync.INSTANCE;
        MovementSync.INSTANCE = movementSync;
        movementSync.entityId = 42;
        movementSync.velocity.set(new Vector3d(1, 2, 3));
    }

    @AfterEach
    void restore() { MovementSync.INSTANCE = previous; }

    private ClientboundExplodePacket explosion(org.cloudburstmc.math.vector.Vector3d impulse) {
        return new ClientboundExplodePacket(org.cloudburstmc.math.vector.Vector3d.ZERO, 4, 0,
                impulse, new Particle(ParticleType.EXPLOSION, null), BuiltinSound.ENTITY_GENERIC_EXPLODE,
                new WeightedList<>(List.of()));
    }

    @Test
    void explosionAddsWithoutMutatingThePreviousSnapshot() {
        Vector3d snapshot = movementSync.velocity.get();
        listener.packetReceived(null, explosion(org.cloudburstmc.math.vector.Vector3d.from(0.5, -1, 2)));
        assertEquals(new Vector3d(1.5, 1, 5), movementSync.velocity.get());
        assertEquals(new Vector3d(1, 2, 3), snapshot);
        listener.packetReceived(null, explosion(org.cloudburstmc.math.vector.Vector3d.from(0.5, -1, 2)));
        assertEquals(new Vector3d(2, 0, 7), movementSync.velocity.get());
    }

    @Test
    void absentExplosionImpulseLeavesVelocityUntouched() {
        Vector3d snapshot = movementSync.velocity.get();
        listener.packetReceived(null, explosion(null));
        assertSame(snapshot, movementSync.velocity.get());
    }

    @Test
    void entityMotionReplacesOnlyOurVelocity() {
        listener.packetReceived(null, new ClientboundSetEntityMotionPacket(41,
                org.cloudburstmc.math.vector.Vector3d.from(4, 5, 6)));
        assertEquals(new Vector3d(1, 2, 3), movementSync.velocity.get());
        listener.packetReceived(null, new ClientboundSetEntityMotionPacket(42,
                org.cloudburstmc.math.vector.Vector3d.from(4, 5, 6)));
        assertEquals(new Vector3d(4, 5, 6), movementSync.velocity.get());
    }
}
