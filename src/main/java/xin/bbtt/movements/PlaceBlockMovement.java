package xin.bbtt.movements;

import org.cloudburstmc.math.vector.Vector3i;
import org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.level.ServerboundPlayerInputPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSwingPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemOnPacket;
import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.movement.Movement;

public class PlaceBlockMovement extends Movement {
    private final Vector3i pos; // Position to place block AT
    private final Vector3i targetPos; // Position to click ON
    private final Direction side;
    private final boolean bridge;
    
    private long startTime;
    private int step = 0;

    public PlaceBlockMovement(Vector3i pos, Vector3i targetPos, Direction side, boolean bridge) {
        this.pos = pos;
        this.targetPos = targetPos;
        this.side = side;
        this.bridge = bridge;
    }

    @Override
    public void init() {
        startTime = System.currentTimeMillis();
        // Calculate the center of the targeted face
        double dx = 0, dy = 0, dz = 0;
        switch (side) {
            case DOWN -> dy = -0.5;
            case UP -> dy = 0.5;
            case NORTH -> dz = -0.5;
            case SOUTH -> dz = 0.5;
            case WEST -> dx = -0.5;
            case EAST -> dx = 0.5;
        }
        
        MovementSync.Instance.lookAt(new Vector3d(
            targetPos.getX() + 0.5 + dx, 
            targetPos.getY() + 0.5 + dy, 
            targetPos.getZ() + 0.5 + dz
        ));
    }

    @Override
    public void onTick() {
        long elapsed = System.currentTimeMillis() - startTime;
        
        if (bridge) {
            switch (step) {
                case 0: // Sneak via input
                    sendInput(true);
                    step = 1;
                    break;
                case 1: // Wait and move to edge
                    if (elapsed > 100) {
                        Vector3d current = MovementSync.Instance.position.get();
                        Vector3d edge = new Vector3d(pos.getX() + 0.5, current.y, pos.getZ() + 0.5);
                        Vector3d diff = new Vector3d(edge).sub(current);
                        if (diff.length() > 0.1) diff.normalize().mul(0.05);
                        MovementSync.Instance.velocity.set(diff);
                        
                        if (elapsed > 300) step = 2;
                    }
                    break;
                case 2: // Place
                    place();
                    step = 3;
                    break;
                case 3: // Stop sneaking
                    if (elapsed > 400) {
                        sendInput(false);
                        setFinished(true);
                    }
                    break;
            }
        } else {
            if (step == 0 && elapsed > 50) {
                place();
                step = 1;
                setFinished(true);
            }
        }
    }

    private void sendInput(boolean sneak) {
        Bot.Instance.getSession().send(new ServerboundPlayerInputPacket(false, false, false, false, false, sneak, false));
    }

    private void place() {
        Bot.Instance.getSession().send(new ServerboundUseItemOnPacket(
            targetPos, side, Hand.MAIN_HAND, 0.5f, 0.5f, 0.5f, false, false, Bot.Instance.getAndIncreaseSequence()
        ));
        Bot.Instance.getSession().send(new ServerboundSwingPacket(Hand.MAIN_HAND));
    }

    @Override
    public long getTime() {
        return 1000;
    }

    @Override
    public void onStop() {
        MovementSync.Instance.velocity.set(new Vector3d(0, 0, 0));
        if (bridge && step < 3) {
            sendInput(false);
        }
    }
}
