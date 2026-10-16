package xin.bbtt.movements;

import org.cloudburstmc.math.vector.Vector3i;
import org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction;
import org.geysermc.mcprotocollib.protocol.data.game.entity.player.Hand;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSwingPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemOnPacket;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;

/** Accessed under the physics batch lock so validation and enqueue cannot straddle a tick. */
final class BlockPlacementAction {
    private long aimedTick = -1;
    private long sentTick = -1;

    boolean sent() { return sentTick >= 0; }

    void aim(Vector3i block, Direction face) {
        var sync = MovementSync.INSTANCE;
        var frame = sync.getPlacementFrame();
        var point = PlacementRay.aim(block, face);
        // directLookAt uses standing eyes; preserve its yaw wrapping while compensating pose.
        if (frame != null) point.y += 1.62 - frame.eyeHeight();
        sync.directLookAt(point);
        aimedTick = frame == null ? 0 : frame.tick();
    }

    boolean tryQueue(Vector3i block, Direction face) {
        var sync = MovementSync.INSTANCE;
        var frame = sync.getPlacementFrame();
        if (sent() || frame == null || frame.tick() <= aimedTick) return false;
        var hit = PlacementRay.cast(sync.getWorld(), frame);
        if (hit == null || !hit.block().equals(block) || hit.face() != face) {
            aim(block, face);
            return false;
        }
        var cursor = hit.cursor();
        sync.queueBeforeMovement(new ServerboundUseItemOnPacket(block, face, Hand.MAIN_HAND,
                (float) cursor.x, (float) cursor.y, (float) cursor.z, false, false,
                Bot.INSTANCE.getAndIncreaseSequence()));
        sync.queueBeforeMovement(new ServerboundSwingPacket(Hand.MAIN_HAND));
        sentTick = frame.tick();
        MovementSync.getLogger().info("Placement queued: tick={}, feet=({},{},{}), rotation=({},{}), block={}, face={}, cursor={}",
                frame.tick(), frame.x(), frame.y(), frame.z(), frame.yaw(), frame.pitch(), block, face, cursor);
        return true;
    }

    boolean confirmed(Vector3i placed) {
        var sync = MovementSync.INSTANCE;
        var frame = sync.getPlacementFrame();
        return sent() && frame != null && frame.tick() > sentTick
                && sync.getWorld().getBlockStateAt(new org.joml.Vector3d(
                        placed.getX(), placed.getY(), placed.getZ())).isSolid();
    }
}
