package xin.bbtt.world;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import lombok.Getter;
import org.geysermc.mcprotocollib.protocol.codec.MinecraftTypes;
import org.cloudburstmc.nbt.NbtMap;
import org.geysermc.mcprotocollib.protocol.data.game.chunk.BitStorage;
import org.geysermc.mcprotocollib.protocol.data.game.chunk.ChunkBiomeData;
import org.geysermc.mcprotocollib.protocol.data.game.chunk.ChunkSection;
import org.geysermc.mcprotocollib.protocol.data.game.chunk.DataPalette;
import org.geysermc.mcprotocollib.protocol.data.game.chunk.palette.PaletteType;
import org.geysermc.mcprotocollib.protocol.data.game.entity.metadata.GlobalPos;
import org.geysermc.mcprotocollib.protocol.data.game.level.HeightmapTypes;
import org.geysermc.mcprotocollib.protocol.data.game.level.LightUpdateData;
import org.geysermc.mcprotocollib.protocol.data.game.level.block.BlockChangeEntry;
import org.geysermc.mcprotocollib.protocol.data.game.level.block.BlockEntityInfo;
import org.geysermc.mcprotocollib.protocol.data.game.level.block.BlockEntityType;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.entity.*;
import org.geysermc.mcprotocollib.protocol.packet.ingame.clientbound.level.*;
import org.jetbrains.annotations.Nullable;
import org.joml.Vector3d;
import org.joml.Vector3i;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.collision.CollisionBox;
import xin.bbtt.collision.CollisionShapeRegistry;
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
    /** Immutable horizontal Minecraft chunk coordinate. */
    public record ChunkPosition(int x, int z) {}

    /** Immutable server-supplied block entity data; {@code nbt} is null when omitted by the server. */
    public record BlockEntitySnapshot(
            int x, int y, int z, BlockEntityType type, @Nullable NbtMap nbt) {}

    /** Immutable server-supplied default spawn data. */
    public record DefaultSpawn(GlobalPos position, float yaw, float pitch) {}

    private record BlockPosition(int x, int y, int z) {}

    private static final class LightSection {
        private final byte[] data;
        private final int uniformValue;

        private LightSection(byte[] data, int uniformValue) {
            this.data = data;
            this.uniformValue = uniformValue;
        }

        private static LightSection from(byte[] source) {
            if (source.length != 2048) {
                throw new IllegalArgumentException("Light section must contain 2048 bytes");
            }
            boolean zero = true;
            boolean full = true;
            for (byte value : source) {
                zero &= value == 0;
                full &= value == (byte) 0xff;
            }
            if (zero) return new LightSection(null, 0);
            if (full) return new LightSection(null, 15);
            return new LightSection(source.clone(), -1);
        }

        private int get(int x, int y, int z) {
            if (data == null) return uniformValue;
            int index = ((y & 15) << 8) | ((z & 15) << 4) | (x & 15);
            int packed = data[index >> 1] & 0xff;
            return (index & 1) == 0 ? packed & 15 : packed >> 4;
        }
    }

    private static final LightSection EMPTY_LIGHT = new LightSection(null, 0);

    private static final class ChunkAuxData {
        private final Map<Integer, LightSection> skyLight = new HashMap<>();
        private final Map<Integer, LightSection> blockLight = new HashMap<>();
        private final Map<BlockPosition, BlockEntitySnapshot> blockEntities = new HashMap<>();
        private final Map<HeightmapTypes, long[]> heightmaps = new EnumMap<>(HeightmapTypes.class);
    }

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
    private final Map<ChunkPosition, ChunkAuxData> chunkAuxData = new HashMap<>();
    private ChunkPosition chunkCacheCenter;
    private Integer chunkCacheRadius;
    private Integer simulationDistance;
    private DefaultSpawn defaultSpawn;
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
                int oldState = section.getBlock(relativeX, relativeY, relativeZ);
                int newState = blockChangeEvent.getChangeTo();
                section.setBlock(relativeX, relativeY, relativeZ, newState);
                var oldBlock = BlockStateParser.Instance.getBlockByStateId(oldState);
                var newBlock = BlockStateParser.Instance.getBlockByStateId(newState);
                ChunkAuxData data = chunkAuxData.get(new ChunkPosition(chunk.x, chunk.z));
                if (data != null) {
                    String oldName = oldBlock == null ? null : oldBlock.getName();
                    String newName = newBlock == null ? null : newBlock.getName();
                    if (!Objects.equals(oldName, newName)) {
                        data.blockEntities.remove(new BlockPosition(
                                entry.getPosition().getX(), entry.getPosition().getY(), entry.getPosition().getZ()));
                    }
                    updateHeightmaps(data, sections,
                            entry.getPosition().getX(), entry.getPosition().getZ());
                }
            }
        }
        finally {
            lock.writeLock().unlock();
        }
    }

    public boolean isOnGround(Vector3d position) {
        Vector3i chunk = getChunk(position);
        if (!chunkLoaded(chunk.x, chunk.z)) return false;

        double halfWidth = (double) 0.6f / 2;
        double tolerance = 1.0e-4;
        return findHighestCollisionTop(
            position.x - halfWidth, position.y - tolerance, position.z - halfWidth,
            position.x + halfWidth, position.y + tolerance, position.z + halfWidth
        ).isPresent();
    }

    public void clear(){
        lock.writeLock().lock();
        try {
            chunks.clear();
            chunkAuxData.clear();
            entities.clear();
            chunkCacheCenter = null;
            chunkCacheRadius = null;
            simulationDistance = null;
            defaultSpawn = null;
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void handleLevelChunkAndLightUpdate(ClientboundLevelChunkWithLightPacket levelChunkWithLightPacket) {
        ByteBuf chunkBuf = Unpooled.wrappedBuffer(levelChunkWithLightPacket.getChunkData());
        try {
            List<ChunkSection> readSections = new ArrayList<>();

            while (chunkBuf.isReadable()) {

                ChunkSection readSection = MinecraftTypes.readChunkSection(chunkBuf, BlockStateParser.getBlockStateRegistrySize(), RegistryDataListener.getBiomeRegistrySize());

                readSections.add(readSection);
            }

            int lowest = Math.floorDiv(RegistryDataListener.getMinWorldY(), 16);
            Map<Integer, ChunkSection> sections = new ConcurrentHashMap<>();
            for (int y = 0;y < readSections.size();y++) {
                sections.put(lowest + y, readSections.get(y));
            }
            ChunkAuxData data = new ChunkAuxData();
            for (Map.Entry<HeightmapTypes, long[]> entry : levelChunkWithLightPacket.getHeightMaps().entrySet()) {
                data.heightmaps.put(entry.getKey(), entry.getValue().clone());
            }
            for (BlockEntityInfo info : levelChunkWithLightPacket.getBlockEntities()) {
                int x = (levelChunkWithLightPacket.getX() << 4) + info.getX();
                int z = (levelChunkWithLightPacket.getZ() << 4) + info.getZ();
                BlockEntitySnapshot snapshot = new BlockEntitySnapshot(
                        x, info.getY(), z, info.getType(), info.getNbt());
                data.blockEntities.put(new BlockPosition(x, info.getY(), z), snapshot);
            }
            lock.writeLock().lock();
            try {
                ChunkPosition position = new ChunkPosition(
                        levelChunkWithLightPacket.getX(), levelChunkWithLightPacket.getZ());
                ChunkAuxData previous = chunkAuxData.get(position);
                if (previous != null) {
                    data.skyLight.putAll(previous.skyLight);
                    data.blockLight.putAll(previous.blockLight);
                }
                applyLightUpdate(data, levelChunkWithLightPacket.getLightData());
                chunks.computeIfAbsent(
                    levelChunkWithLightPacket.getX(), ignored -> new ConcurrentHashMap<>()
                ).put(levelChunkWithLightPacket.getZ(), sections);
                chunkAuxData.put(position, data);
            } finally {
                lock.writeLock().unlock();
            }
            LoadChunkEvent loadChunkEvent = new LoadChunkEvent(
                levelChunkWithLightPacket.getX(), levelChunkWithLightPacket.getZ()
            );
            Bot.INSTANCE.getPluginManager().events().callEvent(loadChunkEvent);
            MovementSync.getLogger().debug(
                "Loaded chunk: ({}, {})",
                levelChunkWithLightPacket.getX(), levelChunkWithLightPacket.getZ()
            );

        } finally {
            chunkBuf.release();
        }
    }

    private static void updateHeightmaps(
            ChunkAuxData data, Map<Integer, ChunkSection> sections, int blockX, int blockZ) {
        int minY = getMinWorldY();
        int maxY = getMaxWorldY();
        int height = maxY - minY + 1;
        int bits = Integer.SIZE - Integer.numberOfLeadingZeros(height);
        int columnIndex = ((blockZ & 15) << 4) | (blockX & 15);
        for (Map.Entry<HeightmapTypes, long[]> entry : data.heightmaps.entrySet()) {
            int highestY = minY - 1;
            for (int y = maxY; y >= minY; y--) {
                ChunkSection section = sections.get(y >> 4);
                if (section == null) continue;
                int stateId;
                synchronized (section) {
                    stateId = section.getBlock(blockX & 15, y & 15, blockZ & 15);
                }
                if (isHeightmapOpaque(entry.getKey(), stateId)) {
                    highestY = y;
                    break;
                }
            }
            BitStorage storage = new BitStorage(bits, 256, entry.getValue());
            storage.set(columnIndex, highestY - minY + 1);
        }
    }

    private static boolean isHeightmapOpaque(HeightmapTypes type, int stateId) {
        var state = BlockStateParser.Instance.parseStateId(stateId);
        if (state.blockName() == null || state.blockName().contains("air")) return false;
        boolean blocksMotion = !state.isPassable();
        boolean blocksMotionOrFluid = blocksMotion || state.isLiquid();
        return switch (type) {
            case WORLD_SURFACE, WORLD_SURFACE_WG -> true;
            case OCEAN_FLOOR, OCEAN_FLOOR_WG -> blocksMotion;
            case MOTION_BLOCKING -> blocksMotionOrFluid;
            case MOTION_BLOCKING_NO_LEAVES -> blocksMotionOrFluid
                    && (state.material() == null || !state.material().contains("leaves"));
        };
    }

    private static void applyLightUpdate(ChunkAuxData data, LightUpdateData update) {
        applyLightType(data.skyLight, update.getSkyYMask(), update.getEmptySkyYMask(), update.getSkyUpdates());
        applyLightType(data.blockLight, update.getBlockYMask(), update.getEmptyBlockYMask(), update.getBlockUpdates());
    }

    private static void applyLightType(
            Map<Integer, LightSection> target, BitSet updateMask, BitSet emptyMask, List<byte[]> updates) {
        int lowestLightSection = Math.floorDiv(RegistryDataListener.getMinWorldY(), 16) - 1;
        int updateIndex = 0;
        for (int bit = updateMask.nextSetBit(0); bit >= 0; bit = updateMask.nextSetBit(bit + 1)) {
            if (updateIndex >= updates.size()) {
                throw new IllegalArgumentException("Light update mask contains more sections than its data list");
            }
            target.put(lowestLightSection + bit, LightSection.from(updates.get(updateIndex++)));
        }
        if (updateIndex != updates.size()) {
            throw new IllegalArgumentException("Light update data contains sections not present in its mask");
        }
        for (int bit = emptyMask.nextSetBit(0); bit >= 0; bit = emptyMask.nextSetBit(bit + 1)) {
            target.put(lowestLightSection + bit, EMPTY_LIGHT);
        }
    }

    public void handleLightUpdate(ClientboundLightUpdatePacket packet) {
        lock.writeLock().lock();
        try {
            ChunkAuxData data = chunkAuxData.get(new ChunkPosition(packet.getX(), packet.getZ()));
            if (data != null) applyLightUpdate(data, packet.getLightData());
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void handleBlockEntityData(ClientboundBlockEntityDataPacket packet) {
        int x = packet.getPosition().getX();
        int y = packet.getPosition().getY();
        int z = packet.getPosition().getZ();
        lock.writeLock().lock();
        try {
            ChunkAuxData data = chunkAuxData.get(new ChunkPosition(x >> 4, z >> 4));
            if (data == null) return;
            BlockPosition position = new BlockPosition(x, y, z);
            data.blockEntities.put(position,
                    new BlockEntitySnapshot(x, y, z, packet.getType(), packet.getNbt()));
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void handleChunksBiomes(ClientboundChunksBiomesPacket packet) {
        int sectionCount = Math.floorDiv(getMaxWorldY() - getMinWorldY() + 1, 16);
        int lowestSection = Math.floorDiv(getMinWorldY(), 16);
        Map<ChunkPosition, List<DataPalette>> updates = new HashMap<>();
        for (ChunkBiomeData biomeData : packet.getChunkBiomeData()) {
            ByteBuf buffer = Unpooled.wrappedBuffer(biomeData.getBuffer());
            try {
                List<DataPalette> palettes = new ArrayList<>(sectionCount);
                for (int i = 0; i < sectionCount; i++) {
                    palettes.add(MinecraftTypes.readDataPalette(
                            buffer, PaletteType.BIOME, RegistryDataListener.getBiomeRegistrySize()));
                }
                if (buffer.isReadable()) {
                    throw new IllegalArgumentException("Biome update contains trailing section data");
                }
                updates.put(new ChunkPosition(biomeData.getX(), biomeData.getZ()), palettes);
            } finally {
                buffer.release();
            }
        }

        lock.writeLock().lock();
        try {
            for (Map.Entry<ChunkPosition, List<DataPalette>> update : updates.entrySet()) {
                ChunkPosition position = update.getKey();
                Map<Integer, Map<Integer, ChunkSection>> xChunks = chunks.get(position.x());
                if (xChunks == null) continue;
                Map<Integer, ChunkSection> sections = xChunks.get(position.z());
                if (sections == null) continue;
                for (int i = 0; i < sectionCount; i++) {
                    int sectionY = lowestSection + i;
                    ChunkSection current = sections.get(sectionY);
                    if (current == null) continue;
                    synchronized (current) {
                        sections.put(sectionY, new ChunkSection(
                                current.getBlockCount(), new DataPalette(current.getBlockData()),
                                update.getValue().get(i)));
                    }
                }
            }
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void handleSetChunkCacheCenter(ClientboundSetChunkCacheCenterPacket packet) {
        lock.writeLock().lock();
        try {
            chunkCacheCenter = new ChunkPosition(packet.getChunkX(), packet.getChunkZ());
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void handleSetChunkCacheRadius(ClientboundSetChunkCacheRadiusPacket packet) {
        lock.writeLock().lock();
        try {
            chunkCacheRadius = packet.getViewDistance();
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void handleSetSimulationDistance(ClientboundSetSimulationDistancePacket packet) {
        lock.writeLock().lock();
        try {
            simulationDistance = packet.getSimulationDistance();
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void handleSetDefaultSpawnPosition(ClientboundSetDefaultSpawnPositionPacket packet) {
        lock.writeLock().lock();
        try {
            defaultSpawn = new DefaultSpawn(packet.getGlobalPos(), packet.getYaw(), packet.getPitch());
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * Returns server-supplied sky light at a block coordinate. Empty means that the chunk or
     * light section is unknown; a present zero means the server explicitly supplied darkness.
     */
    public OptionalInt getSkyLight(int blockX, int blockY, int blockZ) {
        return getLight(blockX, blockY, blockZ, true);
    }

    /**
     * Returns server-supplied block light at a block coordinate. Empty means that the chunk or
     * light section is unknown; a present zero means the server explicitly supplied darkness.
     */
    public OptionalInt getBlockLight(int blockX, int blockY, int blockZ) {
        return getLight(blockX, blockY, blockZ, false);
    }

    private OptionalInt getLight(int blockX, int blockY, int blockZ, boolean sky) {
        if (!isWithinWorldBounds(blockY)) return OptionalInt.empty();
        lock.readLock().lock();
        try {
            ChunkAuxData data = chunkAuxData.get(new ChunkPosition(blockX >> 4, blockZ >> 4));
            if (data == null) return OptionalInt.empty();
            LightSection section = (sky ? data.skyLight : data.blockLight).get(blockY >> 4);
            return section == null ? OptionalInt.empty()
                    : OptionalInt.of(section.get(blockX, blockY, blockZ));
        } finally {
            lock.readLock().unlock();
        }
    }

    /** Returns immutable block-entity data at a position, or empty if unloaded or absent. */
    public Optional<BlockEntitySnapshot> getBlockEntityAt(int blockX, int blockY, int blockZ) {
        lock.readLock().lock();
        try {
            ChunkAuxData data = chunkAuxData.get(new ChunkPosition(blockX >> 4, blockZ >> 4));
            return data == null ? Optional.empty()
                    : Optional.ofNullable(data.blockEntities.get(new BlockPosition(blockX, blockY, blockZ)));
        } finally {
            lock.readLock().unlock();
        }
    }

    /** Returns an immutable snapshot of all block entities in a loaded chunk. */
    public List<BlockEntitySnapshot> getBlockEntitiesInChunk(int chunkX, int chunkZ) {
        lock.readLock().lock();
        try {
            ChunkAuxData data = chunkAuxData.get(new ChunkPosition(chunkX, chunkZ));
            return data == null ? List.of() : List.copyOf(data.blockEntities.values());
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * Returns an absolute world Y from the requested server heightmap, or empty if the chunk or
     * heightmap is unavailable. Coordinates are block X/Z; the map is sampled within that chunk.
     */
    public OptionalInt getHeight(HeightmapTypes type, int blockX, int blockZ) {
        lock.readLock().lock();
        try {
            ChunkAuxData data = chunkAuxData.get(new ChunkPosition(blockX >> 4, blockZ >> 4));
            if (data == null) return OptionalInt.empty();
            long[] packed = data.heightmaps.get(type);
            if (packed == null) return OptionalInt.empty();
            int height = getMaxWorldY() - getMinWorldY() + 1;
            int bits = Integer.SIZE - Integer.numberOfLeadingZeros(height);
            int valuesPerLong = 64 / bits;
            int index = ((blockZ & 15) << 4) | (blockX & 15);
            int cell = index / valuesPerLong;
            if (cell >= packed.length) return OptionalInt.empty();
            int shift = (index % valuesPerLong) * bits;
            long mask = (1L << bits) - 1L;
            return OptionalInt.of((int) ((packed[cell] >>> shift) & mask) + getMinWorldY() - 1);
        } finally {
            lock.readLock().unlock();
        }
    }

    /** Returns the server biome network ID at a block coordinate, or empty if unloaded. */
    public OptionalInt getBiomeId(int blockX, int blockY, int blockZ) {
        lock.readLock().lock();
        try {
            Map<Integer, Map<Integer, ChunkSection>> xChunks = chunks.get(blockX >> 4);
            if (xChunks == null) return OptionalInt.empty();
            Map<Integer, ChunkSection> sections = xChunks.get(blockZ >> 4);
            if (sections == null) return OptionalInt.empty();
            ChunkSection section = sections.get(blockY >> 4);
            if (section == null) return OptionalInt.empty();
            synchronized (section) {
                return OptionalInt.of(section.getBiomeData().get(
                        (blockX & 15) >> 2, (blockY & 15) >> 2, (blockZ & 15) >> 2));
            }
        } finally {
            lock.readLock().unlock();
        }
    }

    /** Returns the registry name of the biome at a block coordinate, or empty if unknown. */
    public Optional<String> getBiomeName(int blockX, int blockY, int blockZ) {
        OptionalInt id = getBiomeId(blockX, blockY, blockZ);
        return id.isPresent() ? RegistryDataListener.getBiomeName(id.getAsInt()) : Optional.empty();
    }

    /** Returns the latest server chunk-cache center, or empty before one is received. */
    public Optional<ChunkPosition> getChunkCacheCenter() {
        lock.readLock().lock();
        try {
            return Optional.ofNullable(chunkCacheCenter);
        } finally {
            lock.readLock().unlock();
        }
    }

    /** Returns the latest server view distance, or empty before one is received. */
    public OptionalInt getChunkCacheRadius() {
        lock.readLock().lock();
        try {
            return chunkCacheRadius == null ? OptionalInt.empty() : OptionalInt.of(chunkCacheRadius);
        } finally {
            lock.readLock().unlock();
        }
    }

    /** Returns the latest server simulation distance, or empty before one is received. */
    public OptionalInt getSimulationDistance() {
        lock.readLock().lock();
        try {
            return simulationDistance == null ? OptionalInt.empty() : OptionalInt.of(simulationDistance);
        } finally {
            lock.readLock().unlock();
        }
    }

    /** Returns the latest server default spawn data, or empty before one is received. */
    public Optional<DefaultSpawn> getDefaultSpawn() {
        lock.readLock().lock();
        try {
            return Optional.ofNullable(defaultSpawn);
        } finally {
            lock.readLock().unlock();
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
        lock.readLock().lock();
        try {
            Map<Integer, Map<Integer, ChunkSection>> xChunks = chunks.get(forgetLevelChunkPacket.getX());
            if (xChunks == null || !xChunks.containsKey(forgetLevelChunkPacket.getZ())) return;
        } finally {
            lock.readLock().unlock();
        }
        UnloadChunkEvent unloadChunkEvent = new UnloadChunkEvent(
            forgetLevelChunkPacket.getX(), forgetLevelChunkPacket.getZ()
        );
        Bot.INSTANCE.getPluginManager().events().callEvent(unloadChunkEvent);
        lock.writeLock().lock();
        try {
            Map<Integer, Map<Integer, ChunkSection>> xChunks = chunks.get(forgetLevelChunkPacket.getX());
            if (xChunks == null) return;
            xChunks.remove(forgetLevelChunkPacket.getZ());
            if (xChunks.isEmpty()) chunks.remove(forgetLevelChunkPacket.getX(), xChunks);
            chunkAuxData.remove(new ChunkPosition(
                    forgetLevelChunkPacket.getX(), forgetLevelChunkPacket.getZ()));
        } finally {
            lock.writeLock().unlock();
        }
        MovementSync.getLogger().debug(
            "Unloaded chunk: ({}, {})",
            forgetLevelChunkPacket.getX(), forgetLevelChunkPacket.getZ()
        );
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

    /** Runs a short compound terrain query against one consistent loaded/block view.
     * Queries must be read-only and must not acquire navigation/player locks. Do not
     * hold this across an entire search: writers need to publish streamed chunks.
     */
    public boolean readTerrain(java.util.function.BooleanSupplier query) {
        lock.readLock().lock();
        try { return query.getAsBoolean(); }
        finally { lock.readLock().unlock(); }
    }

    public boolean chunkLoaded(int chunkX, int chunkZ) {
        lock.readLock().lock();
        try {
            Map<Integer, Map<Integer, ChunkSection>> xChunks = chunks.get(chunkX);
            return xChunks != null && xChunks.containsKey(chunkZ);
        } finally {
            lock.readLock().unlock();
        }
    }

    /** Returns an immutable point-in-time snapshot of every loaded horizontal chunk. */
    public Set<ChunkPosition> loadedChunks() {
        lock.readLock().lock();
        try {
            Set<ChunkPosition> snapshot = new HashSet<>();
            for (Map.Entry<Integer, Map<Integer, Map<Integer, ChunkSection>>> xEntry : chunks.entrySet()) {
                for (Integer chunkZ : xEntry.getValue().keySet()) {
                    snapshot.add(new ChunkPosition(xEntry.getKey(), chunkZ));
                }
            }
            return Set.copyOf(snapshot);
        } finally {
            lock.readLock().unlock();
        }
    }

    public int getBlockAt(Vector3d position) {
        lock.readLock().lock();
        try {
            return blockAtLocked((int) Math.floor(position.x),
                    (int) Math.floor(position.y), (int) Math.floor(position.z));
        } finally {
            lock.readLock().unlock();
        }
    }

    private int blockAtLocked(int x, int y, int z) {
        Map<Integer, Map<Integer, ChunkSection>> xChunks = chunks.get(x >> 4);
        if (xChunks == null) return 0;
        Map<Integer, ChunkSection> sections = xChunks.get(z >> 4);
        if (sections == null) return 0;
        ChunkSection section = sections.get(y >> 4);
        if (section == null) return 0;
        try {
            synchronized (section) {
                return section.getBlock(x & 15, y & 15, z & 15);
            }
        } catch (IndexOutOfBoundsException e) {
            return 0;
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

    /** Returns whether a world-space AABB overlaps any exact block-state collision box. */
    public boolean isBoxColliding(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ) {
        return isBoxCollidingExcluding(
            minX, minY, minZ, maxX, maxY, maxZ, Set.of());
    }

    /** Returns whether an AABB collides after ignoring explicitly changed blocks. */
    public boolean isBoxCollidingExcluding(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ,
            Set<Vector3i> excludedBlocks) {
        if (maxX <= minX || maxY <= minY || maxZ <= minZ) return false;
        CollisionShapeRegistry registry = CollisionShapeRegistry.getInstance();
        int firstX = (int) Math.floor(minX - registry.maxX()) + 1;
        int firstY = Math.max(getMinWorldY(), (int) Math.floor(minY - registry.maxY()) + 1);
        int firstZ = (int) Math.floor(minZ - registry.maxZ()) + 1;
        int lastX = (int) Math.ceil(maxX - registry.minX()) - 1;
        int lastY = Math.min(getMaxWorldY(), (int) Math.ceil(maxY - registry.minY()) - 1);
        int lastZ = (int) Math.ceil(maxZ - registry.minZ()) - 1;

        // Subclasses may provide virtual block data instead of chunk storage.
        boolean direct = getClass() == World.class;
        if (direct) lock.readLock().lock();
        try {
            for (int x = firstX; x <= lastX; x++) {
                for (int y = firstY; y <= lastY; y++) {
                    if (!isWithinWorldBounds(y)) continue;
                    for (int z = firstZ; z <= lastZ; z++) {
                        if (isExcluded(excludedBlocks, x, y, z)) continue;
                        int stateId = direct ? blockAtLocked(x, y, z)
                                : getBlockStateAt(new Vector3d(x, y, z)).stateId();
                        if (registry.intersects(stateId, x, y, z,
                                minX, minY, minZ, maxX, maxY, maxZ)) return true;
                    }
                }
            }
            return false;
        } finally {
            if (direct) lock.readLock().unlock();
        }
    }

    private static boolean isExcluded(Set<Vector3i> excludedBlocks, int x, int y, int z) {
        // Small hypothetical block edits are common here. Avoid allocating a vector
        // and probing an immutable set for every voxel in a swept collision box.
        if (excludedBlocks.isEmpty()) return false;
        if (excludedBlocks.size() > 4) return excludedBlocks.contains(new Vector3i(x, y, z));
        for (Vector3i block : excludedBlocks) {
            if (block.x == x && block.y == y && block.z == z) return true;
        }
        return false;
    }

    /**
     * Finds the highest collision-box top within the supplied world-space search
     * prism. The X/Z bounds describe the actor footprint; Y bounds constrain
     * candidate support surfaces.
     */
    public OptionalDouble findHighestCollisionTop(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ) {
        return findHighestCollisionTopExcluding(
            minX, minY, minZ, maxX, maxY, maxZ, Set.of());
    }

    /** Finds the highest support top while ignoring explicitly changed blocks. */
    public OptionalDouble findHighestCollisionTopExcluding(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ,
            Set<Vector3i> excludedBlocks) {
        if (maxX <= minX || maxY < minY || maxZ <= minZ) return OptionalDouble.empty();
        CollisionShapeRegistry registry = CollisionShapeRegistry.getInstance();
        int firstX = (int) Math.floor(minX - registry.maxX()) + 1;
        int firstY = Math.max(getMinWorldY(), (int) Math.floor(minY - registry.maxY()) + 1);
        int firstZ = (int) Math.floor(minZ - registry.maxZ()) + 1;
        int lastX = (int) Math.ceil(maxX - registry.minX()) - 1;
        int lastY = Math.min(getMaxWorldY(), (int) Math.ceil(maxY - registry.minY()) - 1);
        int lastZ = (int) Math.ceil(maxZ - registry.minZ()) - 1;
        double highest = Double.NEGATIVE_INFINITY;

        // Subclasses may provide virtual block data instead of chunk storage.
        boolean direct = getClass() == World.class;
        if (direct) lock.readLock().lock();
        try {
            for (int x = firstX; x <= lastX; x++) {
                for (int y = firstY; y <= lastY; y++) {
                    for (int z = firstZ; z <= lastZ; z++) {
                        if (isExcluded(excludedBlocks, x, y, z)) continue;
                        int stateId = direct ? blockAtLocked(x, y, z)
                                : getBlockStateAt(new Vector3d(x, y, z)).stateId();
                        for (CollisionBox box : registry.boxesFor(stateId)) {
                            double boxMinX = x + box.minX();
                            double boxMaxX = x + box.maxX();
                            double boxMinZ = z + box.minZ();
                            double boxMaxZ = z + box.maxZ();
                            if (boxMinX >= maxX || boxMaxX <= minX
                                    || boxMinZ >= maxZ || boxMaxZ <= minZ) continue;
                            double top = y + box.maxY();
                            if (top >= minY && top <= maxY && top > highest) highest = top;
                        }
                    }
                }
            }
            return highest == Double.NEGATIVE_INFINITY
                ? OptionalDouble.empty()
                : OptionalDouble.of(highest);
        } finally {
            if (direct) lock.readLock().unlock();
        }
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
