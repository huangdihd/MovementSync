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

public class DigBlockMovement extends Movement implements NavigationBoundMovement {
    private final Vector3i pos;
    private final int toolSlot;
    private long breakTime = -1;
    private long startTime = -1;
    private boolean started = false;
    private Direction side;
    private Long navigationGeneration;

    public DigBlockMovement(Vector3i pos) {
        this(pos, -1);
    }

    public DigBlockMovement(Vector3i pos, int toolSlot) {
        this.pos = pos;
        this.toolSlot = toolSlot;
    }

    @Override
    public void bindNavigationRequest(long generation) {
        this.navigationGeneration = generation;
    }

    @Override
    public void init() {
        if (!isAuthorized()) {
            setFinished(true);
            return;
        }
        if (toolSlot >= 0) MovementSync.INSTANCE.getInventoryManager().switchToSlot(toolSlot);
        BlockState state = MovementSync.INSTANCE.getWorld().getBlockStateAt(new Vector3d(pos.getX(), pos.getY(), pos.getZ()));
        ItemStack held = MovementSync.INSTANCE.getInventoryManager().getHeldItem();
        
        double breakTicks = ToolUtils.calculateBreakTicks(held, state);
        if (breakTicks < 0) {
            setFinished(true);
            return;
        }
        
        this.breakTime = (long)(Math.ceil(breakTicks) * 50);

        // Determine side based on bot position
        Vector3d head = MovementSync.INSTANCE.getHeadPosition();
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

        // Face the block before digging. Servers that validate line of sight
        // reject START_DIGGING when the player isn't looking at the target;
        // directLookAt also sets pitch, so digging a block overhead or
        // underfoot aims at it instead of staring straight ahead.
        MovementSync.INSTANCE.directLookAt(new Vector3d(pos.getX() + 0.5, pos.getY() + 0.5, pos.getZ() + 0.5));
    }

    @Override
    public void onTick() {
        if (!isAuthorized()) {
            setFinished(true);
            return;
        }
        if (!started) {
            MovementSync.getLogger().info(xin.bbtt.mcbot.LangManager.get("movementsync.movement.digging.start", pos.getX(), pos.getY(), pos.getZ(), breakTime));
            if (!runAuthorized(() -> {
                Bot.INSTANCE.getSession().send(new ServerboundPlayerActionPacket(
                    PlayerAction.START_DIGGING, pos, side, Bot.INSTANCE.getAndIncreaseSequence()));
                startTime = System.currentTimeMillis();
                started = true;
            })) {
                setFinished(true);
                return;
            }
            
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
        if (!runAuthorized(() -> {
            Bot.INSTANCE.getSession().send(new ServerboundPlayerActionPacket(
                PlayerAction.FINISH_DIGGING, pos, side, Bot.INSTANCE.getAndIncreaseSequence()));
            setFinished(true);
        })) {
            setFinished(true);
            return;
        }
        MovementSync.getLogger().info(xin.bbtt.mcbot.LangManager.get("movementsync.movement.digging.finished"));
    }

    @Override
    public long getTime() {
        return breakTime + 50;
    }

    @Override
    public void onStop() {
        if (started && !isFinished()) {
            Bot.INSTANCE.getSession().send(new ServerboundPlayerActionPacket(PlayerAction.CANCEL_DIGGING, pos, side, Bot.INSTANCE.getAndIncreaseSequence()));
        }
    }

    private boolean isAuthorized() {
        return navigationGeneration == null
                || MovementSync.INSTANCE.isNavigationRequestCurrent(navigationGeneration);
    }

    private boolean runAuthorized(Runnable action) {
        return navigationGeneration == null
                ? runDirect(action)
                : MovementSync.INSTANCE.runIfNavigationRequestCurrent(navigationGeneration, action);
    }

    private static boolean runDirect(Runnable action) {
        action.run();
        return true;
    }
}
