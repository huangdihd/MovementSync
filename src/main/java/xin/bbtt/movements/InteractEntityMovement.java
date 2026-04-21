package xin.bbtt.movements;

import org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.InteractAction;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundInteractPacket;
import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.movement.Movement;

public class InteractEntityMovement extends Movement {
    private final int entityId;
    private final InteractAction action;
    private final Vector3d target;

    public InteractEntityMovement(int entityId, InteractAction action) {
        this(entityId, action, null);
    }

    public InteractEntityMovement(int entityId, InteractAction action, Vector3d target) {
        this.entityId = entityId;
        this.action = action;
        this.target = target;
    }

    @Override
    public void init() {}

    @Override
    public void onTick() {
        if (action == InteractAction.INTERACT_AT && target != null) {
            Bot.INSTANCE.getSession().send(new ServerboundInteractPacket(entityId, action, (float)target.x, (float)target.y, (float)target.z, Hand.MAIN_HAND, false));
        } else {
            Bot.INSTANCE.getSession().send(new ServerboundInteractPacket(entityId, action, Hand.MAIN_HAND, false));
        }
        setFinished(true);
    }

    @Override
    public long getTime() {
        return -1;
    }

    @Override
    public void onStop() {}
}
