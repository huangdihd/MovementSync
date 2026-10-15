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
import xin.bbtt.physics.PlayerInput;

public class PlaceBlockMovement extends Movement implements NavigationBoundMovement {
    private final Vector3i pos; // Position to place block AT
    private final Vector3i targetPos; // Position to click ON
    private final Direction side;
    private final boolean bridge;
    private final int blockSlot;
    private Long navigationGeneration;
    
    private long startTime;
    private int step = 0;

    public PlaceBlockMovement(Vector3i pos, Vector3i targetPos, Direction side, boolean bridge) {
        this(pos, targetPos, side, bridge, -1);
    }

    public PlaceBlockMovement(
            Vector3i pos, Vector3i targetPos, Direction side, boolean bridge, int blockSlot) {
        this.pos = pos;
        this.targetPos = targetPos;
        this.side = side;
        this.bridge = bridge;
        this.blockSlot = blockSlot;
    }

    @Override
    public void bindNavigationRequest(long generation) {
        this.navigationGeneration = generation;
    }

    @Override
    public void init() {
        if (blockSlot >= 0) MovementSync.INSTANCE.getInventoryManager().switchToSlot(blockSlot);
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
        
        MovementSync.INSTANCE.directLookAt(new Vector3d(
            targetPos.getX() + 0.5 + dx, 
            targetPos.getY() + 0.5 + dy, 
            targetPos.getZ() + 0.5 + dz
        ));
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
        long elapsed = System.currentTimeMillis() - startTime;
        
        if (bridge) {
            switch (step) {
                case 0: // Sneak via input
                    setBridgeInput(false);
                    step = 1;
                    break;
                case 1: // Wait and move to edge
                    if (elapsed > 100) {
                        setBridgeInput(true);
                        
                        if (elapsed > 300) step = 2;
                    }
                    break;
                case 2: // Place
                    setBridgeInput(false);
                    place();
                    step = 3;
                    break;
                case 3: // Stop sneaking
                    if (elapsed > 400) {
                        onStop();
                        setFinished(true);
                    }
                    break;
            }
        } else {
            if (step == 0 && elapsed > 50) {
                place();
                step = 1;
                onStop();
                setFinished(true);
            }
        }
    }

    private void setBridgeInput(boolean approachEdge) {
        Vector3d current = MovementSync.INSTANCE.position.get();
        double deltaX = pos.getX() + 0.5 - current.x;
        double deltaZ = pos.getZ() + 0.5 - current.z;
        double distance = Math.hypot(deltaX, deltaZ);
        double forward = 0;
        double strafe = 0;
        if (approachEdge && distance > 0.1) {
            double yaw = Math.toRadians(MovementSync.INSTANCE.yaw.get());
            forward = (-deltaX * Math.sin(yaw) + deltaZ * Math.cos(yaw)) / distance;
            strafe = (deltaX * Math.cos(yaw) + deltaZ * Math.sin(yaw)) / distance;
        }
        // Quantize the world-space target to keys without changing the placement aim.
        double threshold = Math.sin(Math.PI / 8);
        MovementSync.INSTANCE.setInput(new PlayerInput(
            forward > threshold, forward < -threshold,
            strafe > threshold, strafe < -threshold, false, true, false));
    }
    private void place() {
        MovementSync.getLogger().info(xin.bbtt.mcbot.LangManager.get("movementsync.movement.placing.start", pos.getX(), pos.getY(), pos.getZ(), side));
        // Send use item packet
        Bot.INSTANCE.getSession().send(new ServerboundUseItemOnPacket(
            targetPos, side, Hand.MAIN_HAND, 0.5f, 0.5f, 0.5f, false, false, Bot.INSTANCE.getAndIncreaseSequence()
        ));
        // Swing arm
        Bot.INSTANCE.getSession().send(new ServerboundSwingPacket(Hand.MAIN_HAND));
    }
    @Override
    public long getTime() {
        return 1000;
    }

    @Override
    public void onStop() {
        MovementSync.INSTANCE.clearInput();
    }

    @Override
    public void onPause() {
        onStop();
    }

    @Override
    public void onResume() {
        if (bridge && !isFinished()) setBridgeInput(step == 1);
    }
}
