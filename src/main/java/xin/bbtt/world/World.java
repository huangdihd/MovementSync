package xin.bbtt.world;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import lombok.Getter;
import org.geysermc.mcprotocollib.protocol.codec.MinecraftTypes;
import org.geysermc.mcprotocollib.protocol.data.game.chunk.ChunkSection;
import org.geysermc.mcprotocollib.protocol.data.game.level.block.BlockChangeEntry;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.*;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundBlockUpdatePacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundForgetLevelChunkPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundLevelChunkWithLightPacket;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.ClientboundSectionBlocksUpdatePacket;
import org.joml.Vector3d;
import org.joml.Vector3i;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.Entity.Entity;
import xin.bbtt.MovementSync;
import xin.bbtt.events.BlockChangeEvent;
import xin.bbtt.events.LoadChunkEvent;
import xin.bbtt.events.UnloadChunkEvent;
import xin.bbtt.listeners.RegistryDataListener;
import xin.bbtt.mcbot.Bot;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class World {
    /**
     * Returns true when {@code y} is within the current world height range,
     * dynamically read from the dimension type registry.  Falls back to the
     * vanilla overworld range (-64..319) if the registry has not arrived yet.
     */
    public static boolean isWithinWorldBounds(int y) {
        return y >= getMinWorldY() && y <= getMaxWorldY();
    }

    public static int getMinWorldY() {
        return xin.bbtt.listeners.RegistryDataListener.getMinWorldY();
    }

    public static int getMaxWorldY() {
        return xin.bbtt.listeners.RegistryDataListener.getMaxWorldY();
    }

    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();
    private final Map<Integer, Map<Integer, Map<Integer, ChunkSection>>> chunks = new ConcurrentHashMap<>();
    @Getter
    private final Map<Integer, Entity> entities = new ConcurrentHashMap<>();

    public void applyChange(BlockChangeEntry entry) {
        lock.writeLock().lock();
        try {
            Vector3i chunk = getChunk(entry.getPosition());
            if (!chunks.containsKey(chunk.x)) return;
            Map<Integer, Map<Integer, ChunkSection>> xChunks = chunks.get(chunk.x);
            if (!xChunks.containsKey(chunk.z)) return;
            Map<Integer, ChunkSection> sections = xChunks.get(chunk.z);
            ChunkSection section = sections.get(chunk.y);
            int relativeX = entry.getPosition().getX() & 15;
            int relativeZ = entry.getPosition().getZ() & 15;
            int relativeY = entry.getPosition().getY() & 15;
            synchronized (section) {
                BlockChangeEvent blockChangeEvent = new BlockChangeEvent(new Vector3i(
                        entry.getPosition().getX(),
                        entry.getPosition().getY(),
                        entry.getPosition().getZ()
                ), section.getBlock(relativeX, relativeY, relativeZ), entry.getBlock());
                Bot.INSTANCE.getPluginManager().events().callEvent(blockChangeEvent);
                section.setBlock(relativeX, relativeY, relativeZ, blockChangeEvent.getChangeTo());
            }
        }
        finally {
            lock.writeLock().unlock();
        }
    }

    public boolean isOnGround(Vector3d position) {
        Vector3i chunk = getChunk(position);
        if (!chunkLoaded(chunk.x, chunk.z)) return true;

        Vector3d bottomBlockPos = new Vector3d(position).floor().add(Direction.DOWN.getUnitVector());

        if (position.y > (int)position.y + 0.0001) {
            bottomBlockPos = position;
            bottomBlockPos.y = (int)position.y;
        }

        boolean result = isSolid(bottomBlockPos);
        // North
        if (1 + Math.floor(position.z) - position.z > 0.7) {
            result |= isSolid(new Vector3d(bottomBlockPos).add(Direction.NORTH.getUnitVector()));
        }
        // East
        if (position.x - Math.floor(position.x) > 0.7) {
            result |= isSolid(new Vector3d(bottomBlockPos).add(Direction.EAST.getUnitVector()));
        }
        // South
        if (position.z - Math.floor(position.z) > 0.7) {
            result |=  isSolid(new Vector3d(bottomBlockPos).add(Direction.SOUTH.getUnitVector()));
        }
        // West
        if (1 + Math.floor(position.x) - position.x > 0.7) {
            result |=  isSolid(new Vector3d(bottomBlockPos).add(Direction.WEST.getUnitVector()));
        }
        return result;
    }

    public void clear(){
        chunks.clear();
        entities.clear();
    }

    public void handleLevelChunkAndLightUpdate(ClientboundLevelChunkWithLightPacket levelChunkWithLightPacket) {
        if (!chunks.containsKey(levelChunkWithLightPacket.getX())) {
            chunks.put(levelChunkWithLightPacket.getX(), new ConcurrentHashMap<>());
        }
        ByteBuf chunkBuf = Unpooled.wrappedBuffer(levelChunkWithLightPacket.getChunkData());
        try {
            List<ChunkSection> readSections = new ArrayList<>();

            while (chunkBuf.isReadable()) {

                ChunkSection readSection = MinecraftTypes.readChunkSection(chunkBuf, BlockStateParser.getBlockStateRegistrySize(), RegistryDataListener.getBiomeRegistrySize());

                readSections.add(readSection);
            }

            int lowest = readSections.size() == 16 ? 0 : -4;
            Map<Integer, ChunkSection> sections = new ConcurrentHashMap<>();
            for (int y = 0;y < readSections.size();y++) {
                sections.put(lowest + y, readSections.get(y));
            }
            chunks.get(levelChunkWithLightPacket.getX()).put(levelChunkWithLightPacket.getZ(), sections);
            LoadChunkEvent loadChunkEvent = new LoadChunkEvent(levelChunkWithLightPacket.getX(), levelChunkWithLightPacket.getZ());
            Bot.INSTANCE.getPluginManager().events().callEvent(loadChunkEvent);
            MovementSync.getLogger().debug("Loaded chunk: ({}, {})", levelChunkWithLightPacket.getX(), levelChunkWithLightPacket.getZ());

        } finally {
            chunkBuf.release();
        }
    }

    public void handleBlockUpdatePacket(ClientboundBlockUpdatePacket blockUpdatePacket) {
        BlockChangeEntry blockChangeEntry = blockUpdatePacket.getEntry();
        applyChange(blockChangeEntry);
    }

    public void handleSectionBlocksUpdatePacket(ClientboundSectionBlocksUpdatePacket sectionBlocksUpdatePacket) {
        Arrays.stream(sectionBlocksUpdatePacket.getEntries()).forEach(this::applyChange);
    }

    public void handleForgetLevelChunkPacket(ClientboundForgetLevelChunkPacket forgetLevelChunkPacket) {
        if (!chunks.containsKey(forgetLevelChunkPacket.getX())) return;
        if (!chunks.get(forgetLevelChunkPacket.getX()).containsKey(forgetLevelChunkPacket.getZ())) return;
        UnloadChunkEvent unloadChunkEvent = new UnloadChunkEvent(forgetLevelChunkPacket.getX(), forgetLevelChunkPacket.getZ());
        Bot.INSTANCE.getPluginManager().events().callEvent(unloadChunkEvent);
        chunks.get(forgetLevelChunkPacket.getX()).remove(forgetLevelChunkPacket.getZ());
        MovementSync.getLogger().debug("Unloaded chunk: ({}, {})", forgetLevelChunkPacket.getX(), forgetLevelChunkPacket.getZ());
    }

    public Entity getEntity(int entityId) {
        return entities.get(entityId);
    }

    public void handleAddEntityPacket(ClientboundAddEntityPacket  addEntityPacket) {
        Entity entity = Entity.fromPacket(addEntityPacket);
        entities.put(entity.getEntityId(), entity);
    }

    public void handleMoveEntityPosPacket(ClientboundMoveEntityPosPacket moveEntityPosPacket) {
        Entity entity = entities.get(moveEntityPosPacket.getEntityId());
        if (entity == null) return;
        entity.move(new Vector3d(
                moveEntityPosPacket.getMoveX(),
                moveEntityPosPacket.getMoveY(),
                moveEntityPosPacket.getMoveZ()
        ));
    }

    public void handleMoveEntityRotPacket(ClientboundMoveEntityRotPacket moveEntityRotPacket) {
        Entity entity = entities.get(moveEntityRotPacket.getEntityId());
        if (entity == null) return;
        entity.setYaw(moveEntityRotPacket.getYaw());
        entity.setPitch(moveEntityRotPacket.getPitch());
    }

    public void handleRotateHeadPacket(ClientboundRotateHeadPacket rotateHeadPacket) {
        Entity entity = entities.get(rotateHeadPacket.getEntityId());
        if (entity == null) return;
        entity.setHeadYaw(rotateHeadPacket.getHeadYaw());
    }

    public void handleMoveEntityPosRotPacket(ClientboundMoveEntityPosRotPacket moveEntityPosRotPacket) {
        Entity entity = entities.get(moveEntityPosRotPacket.getEntityId());
        if (entity == null) return;
        entity.move(new Vector3d(
                moveEntityPosRotPacket.getMoveX(),
                moveEntityPosRotPacket.getMoveY(),
                moveEntityPosRotPacket.getMoveZ()
        ));
        entity.setYaw(moveEntityPosRotPacket.getYaw());
        entity.setPitch(moveEntityPosRotPacket.getPitch());
    }

    public void handleRemoveEntitiesPacket(ClientboundRemoveEntitiesPacket removeEntitiesPacket) {
        for (int entityId : removeEntitiesPacket.getEntityIds()) {
            this.entities.remove(entityId);
        }
    }

    public void handleSetEntityMetadataPacket(ClientboundSetEntityDataPacket setEntityMetadataPacket) {
        Entity entity = entities.get(setEntityMetadataPacket.getEntityId());
        if (entity == null) return;
        entity.updateMetadata(setEntityMetadataPacket.getMetadata());
    }

    public Vector3i getChunk(org.cloudburstmc.math.vector.Vector3i blockPosition) {
        return getChunk(new Vector3i(
                        blockPosition.getX(),
                        blockPosition.getY(),
                        blockPosition.getZ()
        ));
    }

    public Vector3i getChunk(Vector3i blockPosition) {
        int chunkX = blockPosition.x >> 4;
        int chunkY = blockPosition.y >> 4;
        int chunkZ = blockPosition.z >> 4;
        return new Vector3i(chunkX, chunkY, chunkZ);
    }

    public Vector3i getChunk(Vector3d blockPosition) {
        return new Vector3i(
                (int)Math.floor(blockPosition.x) >> 4,
                (int)Math.floor(blockPosition.y) >> 4,
                (int)Math.floor(blockPosition.z) >> 4
        );
    }

    public boolean chunkLoaded(int chunkX, int chunkZ) {
        return chunks.containsKey(chunkX) && chunks.get(chunkX).containsKey(chunkZ);
    }

    public int getBlockAt(Vector3d position) {
        Vector3i chunk = getChunk(position);
        int chunkX = chunk.x;
        int chunkY = chunk.y;
        int chunkZ = chunk.z;
        lock.readLock().lock();
        try {
            if (!chunks.containsKey(chunkX)) return 0;
            Map<Integer, Map<Integer, ChunkSection>> xChunks = chunks.get(chunkX);
            if (!xChunks.containsKey(chunkZ)) return 0;
            Map<Integer, ChunkSection> zChunks = xChunks.get(chunkZ);
            if (!zChunks.containsKey(chunkY)) return 0;
            ChunkSection section = zChunks.get(chunkY);
            int relativeX = (int) Math.floor(position.x) & 15;
            int relativeY = (int) Math.floor(position.y) & 15;
            int relativeZ = (int) Math.floor(position.z) & 15;
            try {
                synchronized (section) {
                    return section.getBlock(relativeX, relativeY, relativeZ);
                }
            } catch (IndexOutOfBoundsException e) {
                return 0;
            }
        }
        finally {
            lock.readLock().unlock();
        }
    }

    public xin.bbtt.Block.BlockState getBlockStateAt(Vector3d position) {
        int stateId = getBlockAt(position);
        return BlockStateParser.Instance.parseStateId(stateId);
    }

    public boolean isPassable(Vector3d position) {
        return getBlockStateAt(position).isPassable();
    }

    public boolean isSolid(Vector3d position) {
        return getBlockStateAt(position).isSolid();
    }

    /**
     * Determines if there is a clear line of sight from start to end using a 3D DDA algorithm.
     * This implementation checks if any solid block is intersected by the ray before reaching the end point.
     * @param start The starting point (e.g., bot's eye position).
     * @param end The ending point.
     * @return true if the line of sight is clear, false if obstructed by a solid block.
     */
    public boolean canSee(Vector3d start, Vector3d end) {
        if (start.equals(end)) return true;

        // Current voxel coordinates
        int x = (int) Math.floor(start.x);
        int y = (int) Math.floor(start.y);
        int z = (int) Math.floor(start.z);

        // If starting in a solid block, we can only see points within the same block.
        if (isSolid(new Vector3d(x, y, z))) {
            return (int) Math.floor(end.x) == x && (int) Math.floor(end.y) == y && (int) Math.floor(end.z) == z;
        }

        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double dz = end.z - start.z;

        int stepX = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
        int stepY = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
        int stepZ = dz > 0 ? 1 : (dz < 0 ? -1 : 0);

        // tMax is the distance to the next voxel boundary in each direction, 
        // expressed as a fraction of the total ray length (0.0 to 1.0).
        double tMaxX = (dx == 0) ? Double.POSITIVE_INFINITY : ((stepX > 0 ? x + 1.0 - start.x : start.x - x) / Math.abs(dx));
        double tMaxY = (dy == 0) ? Double.POSITIVE_INFINITY : ((stepY > 0 ? y + 1.0 - start.y : start.y - y) / Math.abs(dy));
        double tMaxZ = (dz == 0) ? Double.POSITIVE_INFINITY : ((stepZ > 0 ? z + 1.0 - start.z : start.z - z) / Math.abs(dz));

        double tDeltaX = (dx == 0) ? Double.POSITIVE_INFINITY : (1.0 / Math.abs(dx));
        double tDeltaY = (dy == 0) ? Double.POSITIVE_INFINITY : (1.0 / Math.abs(dy));
        double tDeltaZ = (dz == 0) ? Double.POSITIVE_INFINITY : (1.0 / Math.abs(dz));

        // Iterate until we reach the target or hit a wall
        for (int i = 0; i < 1000; i++) {
            // Find the closest boundary
            double tNext = Math.min(tMaxX, Math.min(tMaxY, tMaxZ));

            // If the next boundary is at or beyond the end of our segment (t >= 1.0),
            // it means we've reached the target point without hitting any new block.
            if (tNext >= 1.0 - 1e-9) {
                return true;
            }

            // Move to the next voxel
            if (tMaxX == tNext) {
                x += stepX;
                tMaxX += tDeltaX;
            } else if (tMaxY == tNext) {
                y += stepY;
                tMaxY += tDeltaY;
            } else {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }

            // Check if the voxel we just entered is solid.
            // Since tNext < 1.0, this block is definitely between start and end.
            if (isSolid(new Vector3d(x, y, z))) {
                return false;
            }
        }
        return false;
    }

    /**
     * Determines if the bot can see a specific block.
     * @param start The bot's eye position.
     * @param targetBlock The block to check visibility for.
     * @return true if the block is visible, false otherwise.
     */
    public boolean canSeeBlock(Vector3d start, Vector3i targetBlock) {
        Vector3d end = new Vector3d(targetBlock.x + 0.5, targetBlock.y + 0.5, targetBlock.z + 0.5);
        return canSee(start, end);
    }
}
