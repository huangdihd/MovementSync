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
        Vector3d aim = new Vector3d(
            clickedBlock.getX() + 0.5,
            clickedBlock.getY() + 0.5,
            clickedBlock.getZ() + 0.5
        );
        switch (clickedFace) {
            case DOWN -> aim.y -= 0.5;
            case UP -> aim.y += 0.5;
            case NORTH -> aim.z -= 0.5;
            case SOUTH -> aim.z += 0.5;
            case WEST -> aim.x -= 0.5;
            case EAST -> aim.x += 0.5;
        }
        MovementSync.INSTANCE.directLookAt(aim);
    }

    @Override
    public void onTick() {
        if (navigationGeneration == null) {
            onAuthorizedTick();
            return;
        }
        if (!MovementSync.INSTANCE.runIfNavigationRequestCurrent(
                navigationGeneration, this::onAuthorizedTick)) {
            setFinished(true);
        }
    }

    private void onAuthorizedTick() {
        if (!jumped) {
            if (MovementSync.INSTANCE.onGround.get()) {
                MovementSync.INSTANCE.jump();
                jumped = true;
            }
            return;
        }
        double requiredFeetY = blockPos.getY() + 1.0;
        if (MovementSync.INSTANCE.position.get().y + CLEARANCE_EPSILON < requiredFeetY) return;

        MovementSync.getLogger().info(xin.bbtt.mcbot.LangManager.get(
            "movementsync.pathfinding.pillaring",
            blockPos.getX(), blockPos.getY(), blockPos.getZ()));
        Bot.INSTANCE.getSession().send(new ServerboundUseItemOnPacket(
            clickedBlock,
            clickedFace,
            Hand.MAIN_HAND,
            0.5f, 0.5f, 0.5f,
            false,
            false,
            Bot.INSTANCE.getAndIncreaseSequence()
        ));
        Bot.INSTANCE.getSession().send(new ServerboundSwingPacket(Hand.MAIN_HAND));
        setFinished(true);
    }

    @Override
    public long getTime() {
        return 2000L;
    }

    @Override
    public void onStop() {
        Vector3d velocity = MovementSync.INSTANCE.velocity.get();
        if (velocity != null) {
            MovementSync.INSTANCE.velocity.set(new Vector3d(0, velocity.y, 0));
        }
    }
}
