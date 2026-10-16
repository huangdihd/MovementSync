package xin.bbtt.movements;

import org.cloudburstmc.math.vector.Vector3i;
import org.geysermc.mcprotocollib.protocol.data.game.entity.object.Direction;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundSwingPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundUseItemOnPacket;
import org.joml.Vector3d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.MovementSync;
import xin.bbtt.world.World;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class BlockPlacementActionTest {
    @AfterEach void reset() { MovementSync.INSTANCE = null; }

    private MovementSync setup(boolean obstructed) throws Exception {
        MovementSync sync = new MovementSync();
        var field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(sync, new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                boolean solid = (int) Math.floor(p.x) == 0 && (int) Math.floor(p.z) == 0
                        && ((int) Math.floor(p.y) == 63 || obstructed && (int) Math.floor(p.y) == 65);
                return new BlockState(solid ? "stone" : "air", solid ? 1 : 0,
                        Map.of(), solid ? "block" : "empty", 1, true, "stone");
            }
        });
        sync.position.set(new Vector3d(0.5, 65, 0.5));
        return sync;
    }

    @Test void waitsForReportedTickAndQueuesActualFaceCursorOnlyOnce() throws Exception {
        MovementSync sync = setup(false);
        var action = new BlockPlacementAction();
        Vector3i block = Vector3i.from(0, 63, 0);
        action.aim(block, Direction.UP);
        assertFalse(action.tryQueue(block, Direction.UP));
        sync.publishPlacementFrame(0.5, 65, 0.5, sync.yaw.get(), sync.pitch.get(), 1.62);
        action.aim(block, Direction.UP);
        assertFalse(action.tryQueue(block, Direction.UP), "same tick is not a reported rotation");
        sync.publishPlacementFrame(0.5, 65, 0.5, sync.yaw.get(), sync.pitch.get(), 1.62);
        assertTrue(action.tryQueue(block, Direction.UP));
        var packet = assertInstanceOf(ServerboundUseItemOnPacket.class, sync.pollBeforeMovementPacket());
        assertEquals(1f, packet.getCursorY(), 1e-6);
        assertEquals(0.5f, packet.getCursorX(), 1e-6);
        assertEquals(0.5f, packet.getCursorZ(), 1e-6);
        assertInstanceOf(ServerboundSwingPacket.class, sync.pollBeforeMovementPacket());
        assertFalse(action.tryQueue(block, Direction.UP));
        assertNull(sync.pollBeforeMovementPacket());
    }

    @Test void obstructionAndWrongReportedRotationNeverQueue() throws Exception {
        var sync = setup(true);
        var action = new BlockPlacementAction();
        Vector3i block = Vector3i.from(0, 63, 0);
        action.aim(block, Direction.UP);
        sync.publishPlacementFrame(0.5, 65, 0.5, 0, 0, 1.62);
        assertFalse(action.tryQueue(block, Direction.UP));
        sync.publishPlacementFrame(0.5, 65, 0.5, 0, 90, 1.62);
        assertFalse(action.tryQueue(block, Direction.UP));
        assertNull(sync.pollBeforeMovementPacket());
    }

    @Test void wrongFaceNeverQueues() throws Exception {
        var sync = setup(false);
        var action = new BlockPlacementAction();
        var block = Vector3i.from(0, 63, 0);
        action.aim(block, Direction.EAST);
        sync.publishPlacementFrame(0.5, 65, 0.5, 0, 90, 1.62);
        assertFalse(action.tryQueue(block, Direction.EAST));
        assertNull(sync.pollBeforeMovementPacket());
    }
}
