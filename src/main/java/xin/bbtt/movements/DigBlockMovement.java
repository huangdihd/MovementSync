package xin.bbtt.movements;

import org.cloudburstmc.math.vector.Vector3i;
import org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.PlayerAction;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundPlayerActionPacket;
import org.joml.Vector3d;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.inventory.ToolUtils;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.movement.Movement;

public class DigBlockMovement extends Movement {
    private final Vector3i pos;
    private long breakTime = -1;
    private long startTime = -1;
    private boolean started = false;
    private Direction side;

    public DigBlockMovement(Vector3i pos) {
        this.pos = pos;
    }

    @Override
    public void init() {
        BlockState state = MovementSync.Instance.getWorld().getBlockStateAt(new Vector3d(pos.getX(), pos.getY(), pos.getZ()));
        ItemStack held = MovementSync.Instance.getInventoryManager().getHeldItem();
        
        double breakTicks = ToolUtils.calculateBreakTicks(held, state);
        if (breakTicks < 0) {
            setFinished(true);
            return;
        }
        
        this.breakTime = (long)(Math.ceil(breakTicks) * 50);

        // Determine side based on bot position
        Vector3d head = MovementSync.Instance.getHeadPosition();
        double dx = head.x - (pos.getX() + 0.5);
        double dy = head.y - (pos.getY() + 0.5);
        double dz = head.z - (pos.getZ() + 0.5);
        
        if (Math.abs(dy) > Math.max(Math.abs(dx), Math.abs(dz))) {
            side = dy > 0 ? Direction.UP : Direction.DOWN;
        } else if (Math.abs(dx) > Math.abs(dz)) {
            side = dx > 0 ? Direction.EAST : Direction.WEST;
        } else {
            side = dz > 0 ? Direction.SOUTH : Direction.NORTH;
        }
    }

    @Override
    public void onTick() {
        if (!started) {
            Bot.Instance.getSession().send(new ServerboundPlayerActionPacket(PlayerAction.START_DIGGING, pos, side, Bot.Instance.getAndIncreaseSequence()));
            startTime = System.currentTimeMillis();
            started = true;
            
            if (breakTime == 0) {
                finishDigging();
            }
            return;
        }

        if (System.currentTimeMillis() - startTime >= breakTime) {
            finishDigging();
        }
    }

    private void finishDigging() {
        Bot.Instance.getSession().send(new ServerboundPlayerActionPacket(PlayerAction.FINISH_DIGGING, pos, side, Bot.Instance.getAndIncreaseSequence()));
        setFinished(true);
    }

    @Override
    public long getTime() {
        return breakTime + 1000;
    }

    @Override
    public void onStop() {
        if (started && !isFinished()) {
            Bot.Instance.getSession().send(new ServerboundPlayerActionPacket(PlayerAction.CANCEL_DIGGING, pos, side, Bot.Instance.getAndIncreaseSequence()));
        }
    }
}
