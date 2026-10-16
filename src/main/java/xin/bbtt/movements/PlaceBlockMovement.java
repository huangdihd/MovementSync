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
    
    private final BlockPlacementAction action = new BlockPlacementAction();
    private long lastTick = -1;

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
        action.aim(targetPos, side);
    }

    @Override
    public void onTick() {
        if (isFinished()) return;
        if (navigationGeneration == null) {
            onAuthorizedTick();
            return;
        }
        synchronized (MovementSync.INSTANCE) {
            if (!MovementSync.INSTANCE.runIfNavigationRequestCurrent(
                    navigationGeneration, this::onAuthorizedTick)) {
                onStop();
                setFinished(true);
            }
        }
    }

    private void onAuthorizedTick() {
        synchronized (MovementSync.INSTANCE) {
            var frame = MovementSync.INSTANCE.getPlacementFrame();
            if (frame == null || frame.tick() == lastTick) return;
            lastTick = frame.tick();
            if (action.confirmed(pos)) {
                MovementSync.getLogger().info("Placement confirmed: {} at tick={}", pos, frame.tick());
                onStop();
                setFinished(true);
                return;
            }
            if (action.sent()) return;
            if (bridge) setBridgeInput(false);
            if (!action.tryQueue(targetPos, side) && bridge) {
                // Sneaking physics clips motion to supported edges, not an elapsed-time guess.
                setBridgeInput(true);
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
        boolean outsideFace = switch (side) {
            case EAST -> current.x > targetPos.getX() + 1;
            case WEST -> current.x < targetPos.getX();
            case SOUTH -> current.z > targetPos.getZ() + 1;
            case NORTH -> current.z < targetPos.getZ();
            default -> true;
        };
        if (approachEdge && !outsideFace && distance > 0) {
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
    @Override
    public long getTime() {
        return 5000;
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
        if (bridge && !isFinished()) setBridgeInput(false);
    }
}
