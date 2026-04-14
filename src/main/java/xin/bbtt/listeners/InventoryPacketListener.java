package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.network.Session;
import org.geysermc.mcprotocollib.network.event.session.SessionAdapter;
import org.geysermc.mcprotocollib.network.packet.Packet;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.inventory.ClientboundContainerClosePacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.inventory.ClientboundContainerSetContentPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.inventory.ClientboundContainerSetSlotPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.inventory.ClientboundOpenScreenPacket;
import xin.bbtt.MovementSync;

public class InventoryPacketListener extends SessionAdapter {
    @Override
    public void packetReceived(Session session, Packet packet) {
        if (packet instanceof ClientboundContainerSetContentPacket setContentPacket) {
            MovementSync.Instance.inventoryManager.setContainerItems(setContentPacket.getContainerId(), setContentPacket.getItems());
            MovementSync.Instance.inventoryManager.setCurrentStateId(setContentPacket.getStateId());
        } else if (packet instanceof ClientboundContainerSetSlotPacket setSlotPacket) {
            MovementSync.Instance.inventoryManager.setSlot(setSlotPacket.getContainerId(), setSlotPacket.getSlot(), setSlotPacket.getItem());
            MovementSync.Instance.inventoryManager.setCurrentStateId(setSlotPacket.getStateId());
        } else if (packet instanceof ClientboundOpenScreenPacket openScreenPacket) {
            MovementSync.Instance.inventoryManager.setCurrentContainerId(openScreenPacket.getContainerId());
        } else if (packet instanceof ClientboundContainerClosePacket) {
            MovementSync.Instance.inventoryManager.setCurrentContainerId(0);
        }
    }
}
