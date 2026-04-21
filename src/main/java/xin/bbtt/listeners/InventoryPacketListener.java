package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.PacketSendingEvent;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.inventory.ClientboundContainerClosePacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.inventory.ClientboundContainerSetContentPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.inventory.ClientboundContainerSetSlotPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.inventory.ClientboundOpenScreenPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.player.ClientboundSetHeldSlotPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSetCarriedItemPacket;
import xin.bbtt.MovementSync;

public class InventoryPacketListener extends SessionAdapter {
    @Override
    public void packetReceived(Session session, Packet packet) {
        if (packet instanceof ClientboundContainerSetContentPacket p) {
            MovementSync.INSTANCE.getInventoryManager().setContainerItems(p.getContainerId(), p.getItems());
            MovementSync.INSTANCE.getInventoryManager().setCurrentStateId(p.getStateId());
        }

        if (packet instanceof ClientboundContainerSetSlotPacket p) {
            MovementSync.INSTANCE.getInventoryManager().setSlot(p.getContainerId(), p.getSlot(), p.getItem());
            MovementSync.INSTANCE.getInventoryManager().setCurrentStateId(p.getStateId());
        }

        if (packet instanceof ClientboundOpenScreenPacket p) {
            MovementSync.INSTANCE.getInventoryManager().setCurrentContainerId(p.getContainerId());
        }

        if (packet instanceof ClientboundContainerClosePacket) {
            MovementSync.INSTANCE.getInventoryManager().setCurrentContainerId(0);
        }

        if (packet instanceof ClientboundSetHeldSlotPacket p) {
            MovementSync.INSTANCE.getInventoryManager().setHeldSlot(p.getSlot());
        }

        if (packet instanceof ServerboundSetCarriedItemPacket p) {
            MovementSync.INSTANCE.getInventoryManager().setHeldSlot(p.getSlot());
        }
    }
}
