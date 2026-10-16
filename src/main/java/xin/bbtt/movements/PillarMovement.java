package xin.bbtt.movements;

import org.cloudburstmc.math.vector.Vector3i;
import org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSwingPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemOnPacket;
import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.movement.Movement;

/** Jumps clear of the current feet cell, then places a support block beneath it. */
public final class PillarMovement extends Movement implements NavigationBoundMovement {
    private static final double CLEARANCE_EPSILON = 1.0e-7;
    private final Vector3i blockPos;
    private final Vector3i clickedBlock;
    private final Direction clickedFace;
    private final int blockSlot;
    private Long navigationGeneration;
    private boolean jumped;
    private final BlockPlacementAction action = new BlockPlacementAction();
    private long lastTick = -1;

    public PillarMovement(
            Vector3i blockPos,
            Vector3i clickedBlock,
            Direction clickedFace,
            int blockSlot) {
        this.blockPos = blockPos;
        this.clickedBlock = clickedBlock;
        this.clickedFace = clickedFace;
        this.blockSlot = blockSlot;
    }

    @Override
    public void bindNavigationRequest(long generation) {
        this.navigationGeneration = generation;
    }

    @Override
    public void init() {
        if (blockSlot >= 0) MovementSync.INSTANCE.getInventoryManager().switchToSlot(blockSlot);
        action.aim(clickedBlock, clickedFace);
    }

    @Override
    public void onTick() {
        if (isFinished()) return;
        if (navigationGeneration == null) {
            onAuthorizedTick();
            return;
        }
        if (!MovementSync.INSTANCE.runIfNavigationRequestCurrent(
                navigationGeneration, this::onAuthorizedTick)) {
            onStop();
            setFinished(true);
        }
    }

    private void onAuthorizedTick() {
        synchronized (MovementSync.INSTANCE) {
            var frame = MovementSync.INSTANCE.getPlacementFrame();
            if (frame != null && frame.tick() == lastTick) return;
            if (frame != null) lastTick = frame.tick();
            if (action.confirmed(blockPos) && MovementSync.INSTANCE.onGround.get()) {
                MovementSync.getLogger().info("Pillar placement confirmed: {}", blockPos);
                onStop();
                setFinished(true);
                return;
            }
            if (action.sent()) return;
            if (!jumped) {
                if (MovementSync.INSTANCE.onGround.get()) {
                    MovementSync.INSTANCE.jump();
                    jumped = true;
                }
                return;
            }
            double requiredFeetY = blockPos.getY() + 1.0;
            if (frame == null || frame.y() + CLEARANCE_EPSILON < requiredFeetY) return;
            action.tryQueue(clickedBlock, clickedFace);
        }
    }

    @Override
    public long getTime() {
        return 2000L;
    }

    @Override
    public void onStop() {
        MovementSync.INSTANCE.clearInput();
    }

    @Override
    public void onPause() {
        onStop();
        jumped = false;
    }
}
