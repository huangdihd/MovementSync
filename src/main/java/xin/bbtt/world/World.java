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
                Bot.Instance.getPluginManager().events().callEvent(blockChangeEvent);
                section.setBlock(relativeX, relativeY, relativeZ, blockChangeEvent.getChangeTo());
                section.setBlock(relativeX, relativeY, relativeZ, entry.getBlock());
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
            Bot.Instance.getPluginManager().events().callEvent(loadChunkEvent);
            MovementSync.Instance.getLogger().debug("Loaded chunk: ({}, {})", levelChunkWithLightPacket.getX(), levelChunkWithLightPacket.getZ());

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
        Bot.Instance.getPluginManager().events().callEvent(unloadChunkEvent);
        chunks.get(forgetLevelChunkPacket.getX()).remove(forgetLevelChunkPacket.getZ());
        MovementSync.Instance.getLogger().debug("Unloaded chunk: ({}, {})", forgetLevelChunkPacket.getX(), forgetLevelChunkPacket.getZ());
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
}
