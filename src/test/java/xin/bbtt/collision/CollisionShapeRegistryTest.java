package xin.bbtt.collision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Exact 1.21.11 state collision shapes, keyed by protocol state ID. */
final class CollisionShapeRegistryTest {
    private static final int AIR = 0;
    private static final int STONE = 1;
    private static final int OPEN_NORTH_LEFT_JUNGLE_DOOR = 13995;

    @Test
    void loadsEveryProtocolStateFromThePinnedRegistry() {
        CollisionShapeRegistry registry = CollisionShapeRegistry.getInstance();

        assertEquals(29671, registry.stateCount());
        assertTrue(registry.boxesFor(AIR).isEmpty());
        assertEquals(List.of(new CollisionBox(0, 0, 0, 1, 1, 1)), registry.boxesFor(STONE));
    }

    @Test
    void openDoorCollisionDependsOnApproachThroughItsRotatedPanel() {
        CollisionShapeRegistry registry = CollisionShapeRegistry.getInstance();
        List<CollisionBox> boxes = registry.boxesFor(OPEN_NORTH_LEFT_JUNGLE_DOOR);

        // Paper 1.21.11 DoorBlock: north + open + left rotates clockwise to
        // the east shape, a 3/16-thick panel against the west side of the cell.
        assertEquals(List.of(new CollisionBox(0, 0, 0, 3.0 / 16.0, 1, 1)), boxes);

        // Walking north/south through the block centre clears the open panel.
        assertFalse(registry.collidesAt(OPEN_NORTH_LEFT_JUNGLE_DOOR, 0.5, 0.5, 0.5));
        // Approaching through the west-side panel still collides.
        assertTrue(registry.collidesAt(OPEN_NORTH_LEFT_JUNGLE_DOOR, 0.1, 0.5, 0.5));
    }

    @Test
    void rejectsRegistryVersionMismatchAndInvalidCoordinates() throws Exception {
        byte[] wrongStateCount = encodedRegistry(2,
            new float[][]{{0, 0, 0, 1, 1, 1}}, new float[][]{});
        assertThrows(IOException.class, () -> CollisionShapeRegistry.read(
            new ByteArrayInputStream(wrongStateCount), 1));

        byte[] nonFinite = encodedRegistry(1,
            new float[][]{{Float.NaN, 0, 0, 1, 1, 1}});
        assertThrows(IOException.class, () -> CollisionShapeRegistry.read(
            new ByteArrayInputStream(nonFinite), 1));

        byte[] reversed = encodedRegistry(1,
            new float[][]{{1, 0, 0, 0, 1, 1}});
        assertThrows(IOException.class, () -> CollisionShapeRegistry.read(
            new ByteArrayInputStream(reversed), 1));
    }

    private static byte[] encodedRegistry(int stateCount, float[][]... states) throws IOException {
        ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        try (DataOutputStream output = new DataOutputStream(bytes)) {
            output.writeInt(0x4D534353);
            output.writeInt(1);
            output.writeInt(stateCount);
            for (float[][] boxes : states) {
                output.writeByte(boxes.length);
                for (float[] box : boxes) {
                    for (float coordinate : box) output.writeFloat(coordinate);
                }
            }
        }
        return bytes.toByteArray();
    }

    @Test
    void playerBoxIntersectionUsesAllSubBoxesNotTheCoarseBlockFlag() {
        CollisionShapeRegistry registry = CollisionShapeRegistry.getInstance();

        assertFalse(registry.intersects(
            OPEN_NORTH_LEFT_JUNGLE_DOOR, 976, 72, 998,
            976.20, 72.0, 998.20, 976.80, 73.8, 998.80));
        assertTrue(registry.intersects(
            OPEN_NORTH_LEFT_JUNGLE_DOOR, 976, 72, 998,
            975.90, 72.0, 998.20, 976.20, 73.8, 998.80));
    }
}
