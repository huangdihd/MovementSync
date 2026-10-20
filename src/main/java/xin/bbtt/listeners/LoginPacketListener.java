package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundLoginPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.ClientboundRespawnPacket;
import xin.bbtt.MovementSync;
import org.geysermc.mcprotocollib.protocol.packet.common.serverbound.ServerboundClientInformationPacket;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.HandPreference;
import org.geysermc.mcprotocollib.protocol.data.game.setting.ChatVisibility;
import org.geysermc.mcprotocollib.protocol.data.game.setting.ParticleStatus;
import org.geysermc.mcprotocollib.protocol.data.game.setting.SkinPart;

import java.util.List;

/**
 * Captures the current dimension index from login and respawn packets,
 * so {@code RegistryDataListener} can return the correct min/max world Y.
 */
public class LoginPacketListener extends SessionAdapter {
    private String currentWorldName;
    private Integer currentDimension;

    @Override
    public void packetReceived(Session session, Packet packet) {
        if (packet instanceof ClientboundLoginPacket login) {
            // Vanilla sends these in configuration; the common packet is also valid in play.
            // The plugin owns only play login, so repeat it here on every server join.
            session.send(new ServerboundClientInformationPacket(
                    "en_us", 12, ChatVisibility.FULL, true, List.of(SkinPart.values()),
                    HandPreference.RIGHT_HAND, false, true, ParticleStatus.ALL));
            String worldName = login.getCommonPlayerSpawnInfo().getWorldName().asString();
            if (MovementSync.INSTANCE != null) {
                MovementSync.INSTANCE.getWorld().clear();
                MovementSync.INSTANCE.setRecoveryDimension(worldName);
            }
            currentWorldName = worldName;
            int dim = login.getCommonPlayerSpawnInfo().getDimension();
            currentDimension = dim;
            RegistryDataListener.setCurrentDimension(dim);
            MovementSync.getLogger().debug("Login: dimension index = {}", dim);
        } else if (packet instanceof ClientboundRespawnPacket respawn) {
            String worldName = respawn.getCommonPlayerSpawnInfo().getWorldName().asString();
            int dim = respawn.getCommonPlayerSpawnInfo().getDimension();
            if (MovementSync.INSTANCE != null) {
                MovementSync.INSTANCE.onPlayerRespawn();
                MovementSync.INSTANCE.suspendNavigationForRecovery();
                if (!worldName.equals(currentWorldName) || !Integer.valueOf(dim).equals(currentDimension)) {
                    MovementSync.INSTANCE.getWorld().clear();
                }
                MovementSync.INSTANCE.setRecoveryDimension(worldName);
            }
            currentWorldName = worldName;
            currentDimension = dim;
            RegistryDataListener.setCurrentDimension(dim);
            MovementSync.getLogger().debug("Respawn: dimension index = {}", dim);
        }
    }
}
