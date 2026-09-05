package xin.bbtt.listeners;

import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.event.EventHandler;
import xin.bbtt.mcbot.event.Listener;
import xin.bbtt.mcbot.events.ServerChangeEvent;

public class ServerChangeListener implements Listener {
    @EventHandler
    public void onServerChange(ServerChangeEvent event) {
        MovementSync.INSTANCE.cancelNavigation();
        MovementSync.INSTANCE.onGround.set(true);
        MovementSync.INSTANCE.velocity.set(new Vector3d());
        MovementSync.INSTANCE.getWorld().clear();
    }
}
