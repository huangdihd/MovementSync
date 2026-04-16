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
            MovementSync.Instance.getInventoryManager().setContainerItems(p.getContainerId(), p.getItems());
            MovementSync.Instance.getInventoryManager().setCurrentStateId(p.getStateId());
            return;
        }
        
        if (packet instanceof ClientboundContainerSetSlotPacket p) {
            MovementSync.Instance.getInventoryManager().setSlot(p.getContainerId(), p.getSlot(), p.getItem());
            MovementSync.Instance.getInventoryManager().setCurrentStateId(p.getStateId());
            return;
        }
        
        if (packet instanceof ClientboundOpenScreenPacket p) {
            MovementSync.Instance.getInventoryManager().setCurrentContainerId(p.getContainerId());
            return;
        }
        
        if (packet instanceof ClientboundContainerClosePacket) {
            MovementSync.Instance.getInventoryManager().setCurrentContainerId(0);
            return;
        }
        
        if (packet instanceof ClientboundSetHeldSlotPacket p) {
            MovementSync.Instance.getInventoryManager().setHeldSlot(p.getSlot());
        }
    }

    @Override
    public void packetSending(PacketSendingEvent event) {
        if (event.getPacket() instanceof ServerboundSetCarriedItemPacket p) {
            MovementSync.Instance.getInventoryManager().setHeldSlot(p.getSlot());
        }
    }
}
