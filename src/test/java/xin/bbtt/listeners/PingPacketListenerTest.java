package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.protocol.packet.common.clientbound.ClientboundPingPacket;
import org.geysermc.mcprotocollib.protocol.packet.common.serverbound.ServerboundPongPacket;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;

class PingPacketListenerTest {
    @Test
    void pingIsAnsweredWithAPongCarryingTheSameId() {
        List<Object> packets = new ArrayList<>();
        Session session = (Session) Proxy.newProxyInstance(
            Session.class.getClassLoader(),
            new Class<?>[]{Session.class},
            (proxy, method, args) -> {
                if (method.getName().equals("send")) packets.add(args[0]);
                return null;
            });

        new PingPacketListener().packetReceived(session, new ClientboundPingPacket(1234));

        assertEquals(1, packets.size());
        assertEquals(1234, assertInstanceOf(ServerboundPongPacket.class, packets.get(0)).getId());
    }
}
