package xin.bbtt.collision;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import xin.bbtt.Block.BlockStateParser;

/**
 * Pinned Paper 1.21.11 state-derived collision VoxelShapes indexed by protocol
 * state ID. The resource uses EmptyBlockGetter + CollisionContext.empty();
 * entity/world-dependent overrides (powder snow, scaffolding, moving piston)
 * require runtime context that a state ID alone cannot encode.
 */
public final class CollisionShapeRegistry {
    private static final int MAGIC = 0x4D534353; // MSCS
    private static final int FORMAT_VERSION = 1;
    private static final String RESOURCE = "collision_shapes.bin";
    private static final CollisionShapeRegistry INSTANCE = load();

    private final List<CollisionBox>[] boxesByState;
    private final double minX;
    private final double minY;
    private final double minZ;
    private final double maxX;
    private final double maxY;
    private final double maxZ;

    private CollisionShapeRegistry(
            List<CollisionBox>[] boxesByState,
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ) {
        this.boxesByState = boxesByState;
        this.minX = minX;
        this.minY = minY;
        this.minZ = minZ;
        this.maxX = maxX;
        this.maxY = maxY;
        this.maxZ = maxZ;
    }

    public static CollisionShapeRegistry getInstance() {
        return INSTANCE;
    }

    public int stateCount() {
        return boxesByState.length;
    }

    public double minX() { return minX; }
    public double minY() { return minY; }
    public double minZ() { return minZ; }
    public double maxX() { return maxX; }
    public double maxY() { return maxY; }
    public double maxZ() { return maxZ; }

    public List<CollisionBox> boxesFor(int stateId) {
        if (stateId < 0 || stateId >= boxesByState.length) return List.of();
        return boxesByState[stateId];
    }

    public boolean collidesAt(int stateId, double localX, double localY, double localZ) {
        for (CollisionBox box : boxesFor(stateId)) {
            if (box.contains(localX, localY, localZ)) return true;
        }
        return false;
    }

    public boolean intersects(
            int stateId, int blockX, int blockY, int blockZ,
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ) {
        double localMinX = minX - blockX;
        double localMinY = minY - blockY;
        double localMinZ = minZ - blockZ;
        double localMaxX = maxX - blockX;
        double localMaxY = maxY - blockY;
        double localMaxZ = maxZ - blockZ;
        for (CollisionBox box : boxesFor(stateId)) {
            if (box.intersects(localMinX, localMinY, localMinZ,
                    localMaxX, localMaxY, localMaxZ)) return true;
        }
        return false;
    }

    private static CollisionShapeRegistry load() {
        InputStream raw = Objects.requireNonNull(
            CollisionShapeRegistry.class.getClassLoader().getResourceAsStream(RESOURCE),
            "Missing " + RESOURCE);
        try {
            return read(raw, BlockStateParser.getBlockStateRegistrySize());
        } catch (IOException error) {
            throw new ExceptionInInitializerError(error);
        }
    }

    @SuppressWarnings("unchecked")
    static CollisionShapeRegistry read(InputStream raw, int expectedStateCount) throws IOException {
        try (DataInputStream input = new DataInputStream(new BufferedInputStream(raw))) {
            int magic = input.readInt();
            if (magic != MAGIC) throw new IOException("Invalid collision registry magic");
            int version = input.readInt();
            if (version != FORMAT_VERSION) {
                throw new IOException("Unsupported collision registry format " + version);
            }
            int stateCount = input.readInt();
            if (stateCount <= 0 || stateCount != expectedStateCount) {
                throw new IOException("Collision registry state count " + stateCount
                    + " does not match protocol registry " + expectedStateCount);
            }
            List<CollisionBox>[] states = (List<CollisionBox>[]) new List<?>[stateCount];
            double minX = Double.POSITIVE_INFINITY;
            double minY = Double.POSITIVE_INFINITY;
            double minZ = Double.POSITIVE_INFINITY;
            double maxX = Double.NEGATIVE_INFINITY;
            double maxY = Double.NEGATIVE_INFINITY;
            double maxZ = Double.NEGATIVE_INFINITY;
            for (int stateId = 0; stateId < stateCount; stateId++) {
                int count = input.readUnsignedByte();
                if (count == 0) {
                    states[stateId] = List.of();
                    continue;
                }
                List<CollisionBox> boxes = new ArrayList<>(count);
                for (int index = 0; index < count; index++) {
                    CollisionBox box = new CollisionBox(
                        input.readFloat(), input.readFloat(), input.readFloat(),
                        input.readFloat(), input.readFloat(), input.readFloat());
                    if (!isValid(box)) {
                        throw new IOException("Invalid collision box for state " + stateId
                            + " at index " + index + ": " + box);
                    }
                    boxes.add(box);
                    minX = Math.min(minX, box.minX());
                    minY = Math.min(minY, box.minY());
                    minZ = Math.min(minZ, box.minZ());
                    maxX = Math.max(maxX, box.maxX());
                    maxY = Math.max(maxY, box.maxY());
                    maxZ = Math.max(maxZ, box.maxZ());
                }
                states[stateId] = Collections.unmodifiableList(boxes);
            }
            if (input.read() != -1) throw new IOException("Trailing bytes in collision registry");
            if (!Double.isFinite(minX) || !Double.isFinite(maxX)) {
                throw new IOException("Collision registry contains no boxes");
            }
            return new CollisionShapeRegistry(states, minX, minY, minZ, maxX, maxY, maxZ);
        }
    }

    private static boolean isValid(CollisionBox box) {
        return Double.isFinite(box.minX()) && Double.isFinite(box.minY())
            && Double.isFinite(box.minZ()) && Double.isFinite(box.maxX())
            && Double.isFinite(box.maxY()) && Double.isFinite(box.maxZ())
            && box.minX() < box.maxX() && box.minY() < box.maxY()
            && box.minZ() < box.maxZ();
    }
}
