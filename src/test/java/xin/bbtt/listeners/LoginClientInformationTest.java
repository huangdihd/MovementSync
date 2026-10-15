package xin.bbtt.listeners;

import net.kyori.adventure.key.Key;
import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.GameMode;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.HandPreference;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerSpawnInfo;
import org.geysermc.mcprotocollib.protocol.data.game.setting.ChatVisibility;
import org.geysermc.mcprotocollib.protocol.data.game.setting.ParticleStatus;
import org.geysermc.mcprotocollib.protocol.data.game.setting.SkinPart;
import org.geysermc.mcprotocollib.protocol.packet.common.serverbound.ServerboundClientInformationPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundLoginPacket;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

final class LoginClientInformationTest {
    @Test
    void everyLoginReportsViewDistanceAndVanillaDefaults() {
        List<Object> packets = new ArrayList<>();
        Session session = (Session) Proxy.newProxyInstance(Session.class.getClassLoader(),
                new Class<?>[]{Session.class}, (proxy, method, arguments) -> {
                    if (method.getName().equals("send")) packets.add(arguments[0]);
                    return null;
                });
        Key world = Key.key("minecraft:overworld");
        ClientboundLoginPacket login = new ClientboundLoginPacket(1, false, new Key[]{world},
                20, 12, 10, false, true, false,
                new PlayerSpawnInfo(0, world, 0, GameMode.SURVIVAL, null, false, true, null, 0, 63), false);
        LoginPacketListener listener = new LoginPacketListener();
        listener.packetReceived(session, login);
        listener.packetReceived(session, login);
        assertEquals(2, packets.size());
        ServerboundClientInformationPacket information = assertInstanceOf(
                ServerboundClientInformationPacket.class, packets.get(0));
        assertEquals("en_us", information.getLocale());
        assertEquals(12, information.getRenderDistance());
        assertEquals(ChatVisibility.FULL, information.getChatVisibility());
        assertTrue(information.isUseChatColors());
        assertEquals(List.of(SkinPart.values()), information.getVisibleParts());
        assertEquals(HandPreference.RIGHT_HAND, information.getMainHand());
        assertFalse(information.isTextFilteringEnabled());
        assertTrue(information.isAllowsListing());
        assertEquals(ParticleStatus.ALL, information.getParticleStatus());
    }
}
