package xin.bbtt.listeners;

import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.*;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.event.EventHandler;
import xin.bbtt.mcbot.event.Listener;
import xin.bbtt.mcbot.events.ReceivePacketEvent;

public class EntityPacketListener implements Listener {

    @EventHandler
    public void OnAddEntities(ReceivePacketEvent<ClientboundAddEntityPacket> receivePacketEvent) {
        MovementSync.Instance.getWorld().handleAddEntityPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnMoveEntityPosition(ReceivePacketEvent<ClientboundMoveEntityPosPacket> receivePacketEvent) {
        MovementSync.Instance.getWorld().handleMoveEntityPosPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnMoveEntityRotation(ReceivePacketEvent<ClientboundMoveEntityRotPacket> receivePacketEvent) {
        MovementSync.Instance.getWorld().handleMoveEntityRotPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnMoveEntityPositronRotation(ReceivePacketEvent<ClientboundMoveEntityPosRotPacket> receivePacketEvent) {
        MovementSync.Instance.getWorld().handleMoveEntityPosRotPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnMoveEntityRotateHead(ReceivePacketEvent<ClientboundRotateHeadPacket> receivePacketEvent) {
        MovementSync.Instance.getWorld().handleRotateHeadPacket(receivePacketEvent.getPacket());
    }

    @EventHandler
    public void OnRemoveEntity(ReceivePacketEvent<ClientboundRemoveEntitiesPacket> receivePacketEvent) {
        MovementSync.Instance.getWorld().handleRemoveEntitiesPacket(receivePacketEvent.getPacket());
    }
}
