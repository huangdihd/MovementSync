package xin.bbtt.flight;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HexFormat;
import java.util.List;
import java.util.Map;
import java.util.OptionalInt;
import java.util.UUID;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.Entity.Entity;
import xin.bbtt.MovementSync;
import xin.bbtt.collision.CollisionShapeRegistry;
import xin.bbtt.physics.ElytraFlightController;
import xin.bbtt.physics.GlidingPhysics;
import xin.bbtt.physics.MovementParameters;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PhysicsStepper;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.physics.WalkingPhysics;
import xin.bbtt.world.World;

/** Regression for the immutable 64-tick production updateMotion capture. */
final class N180UpdateMotionCaptureReplayTest {
    private static final String RESOURCE = "xin/bbtt/flight/n180-update-motion-64.n180";
    private static final byte[] SHA256 = HexFormat.of().parseHex(
            "6d48231e6dc40e261567d1951109df3e81fb92549b6d93f04c632842a8e1192b");
    private static final int MAGIC = 0x4e313830;
    private static final int VERSION = 2;
    private static final int FIREWORK_ID = 0x180;
    private static final int PLAYER_ID = 42;

    @Test
    void realCaptureExactlyReplaysPhysicsAttachmentAndGotoCollisionPolicy() throws Exception {
        byte[] bytes = resourceBytes();
        assertArrayEquals(SHA256, MessageDigest.getInstance("SHA-256").digest(bytes),
                "fixture bytes must remain the reviewed production capture");
        Incident incident = Incident.read(bytes);

        assertEquals(VERSION, incident.schema());
        assertEquals(CollisionShapeRegistry.getInstance().stateCount(), incident.registryStateCount());
        assertEquals(64, incident.frames().size());
        assertEquals(29200, incident.frames().get(0).sequence());
        assertEquals(29263, incident.frames().get(63).sequence());
        assertEquals(29201, incident.frames().get(0).physicsTick());
        assertEquals(29264, incident.frames().get(63).physicsTick());
        assertEquals(Trigger.GLIDING_COLLIDED_X, incident.trigger());
        assertEquals(29263, incident.triggerSequence());
        assertEquals(29264, incident.triggerPhysicsTick());
        assertEquals(incident.continuity(), incident.triggerContinuity());
        assertEquals(-1, incident.rawFlags());
        assertEquals(-1, incident.effectiveFlags());
        assertTrue(incident.glidingBefore());
        assertTrue(incident.glidingAfter());

        MovementSync previous = MovementSync.INSTANCE;
        MovementSync sync = new MovementSync();
        sync.entityId = PLAYER_ID;
        try {
            for (int index = 0; index < incident.frames().size(); index++) {
                Frame frame = incident.frames().get(index);
                assertEquals(29200L + index, frame.sequence(), "sequence continuity");
                assertEquals(29201L + index, frame.physicsTick(), "physics-tick continuity");
                assertEquals(incident.continuity(), frame.continuity(), "capture continuity token");
                assertEquals(index >= 48, frame.attached(), "only the final 16 frames are attached");
                assertFalse(frame.transcriptOverflow(), "frame " + frame.sequence() + " transcript overflow");
                assertEquals(0, frame.droppedEvents(), "frame " + frame.sequence() + " dropped transcript events");

                TranscriptWorld world = TranscriptWorld.from(frame, incident.registryStateCount());
                installWorld(sync, world);
                sync.setGliding(frame.gliding());
                installAttachment(world, frame.attached());
                boolean attached = sync.hasAttachedFirework();
                assertEquals(frame.attached(), attached,
                        "attachment must be sampled through MovementSync for frame " + frame.sequence());

                PhysicsStepper stepper = frame.gliding()
                        ? new GlidingPhysics(frame.yaw(), frame.pitch(), attached,
                                frame.input(), frame.parameters())
                        : new WalkingPhysics(frame.input(), frame.yaw(), frame.jumpAllowed(), frame.parameters());
                PhysicsResult actual = stepper.step(world, frame.state(), frame.halfWidth(), frame.height());
                assertPhysicsResultBits(frame.expected(), actual, frame.sequence());
            }

            Frame contact = incident.frames().get(62);
            assertTrue(contact.attached());
            assertTrue(contact.expected().onGround(), "the tick before the trigger is the real first contact");
            assertFalse(contact.expected().collidedX(), "policy proof must not rely on collision rejection");
            assertFalse(contact.expected().collidedZ(), "policy proof must not rely on collision rejection");
            TranscriptWorld contactWorld = TranscriptWorld.from(
                    incident.frames(), incident.registryStateCount());
            installWorld(sync, contactWorld);
            sync.setGliding(true);
            installAttachment(contactWorld, true);
            boolean attached = sync.hasAttachedFirework();
            assertTrue(attached, "policy input must come from the real attached firework metadata");

            var control = new ElytraFlightController.Decision(contact.yaw(), contact.pitch(), false,
                    ElytraFlightController.Phase.LANDING);
            var replay = LocalGotoController.replay(contactWorld, contact.parameters(), contact.state(),
                    control, attached, 40, contact.expected().position(), true);
            assertTrue(replay.steps().isEmpty(),
                    "real first contact with attached thrust is not a stopped terminal landing");
        } finally {
            MovementSync.INSTANCE = previous;
            if (sync.movementService != null) sync.movementService.shutdownNow();
        }
    }

    private static byte[] resourceBytes() throws IOException {
        try (InputStream input = N180UpdateMotionCaptureReplayTest.class.getClassLoader()
                .getResourceAsStream(RESOURCE)) {
            if (input == null) throw new IOException("Missing test fixture " + RESOURCE);
            return input.readAllBytes();
        }
    }

    private static void installWorld(MovementSync sync, World world) throws ReflectiveOperationException {
        Field field = MovementSync.class.getDeclaredField("world");
        field.setAccessible(true);
        field.set(sync, world);
    }

    private static void installAttachment(World world, boolean attached) {
        world.getEntities().clear();
        if (!attached) return;
        Entity rocket = new Entity(FIREWORK_ID, new UUID(0, FIREWORK_ID), EntityType.FIREWORK_ROCKET,
                0, 0, 0, 0, 0, 0, new Vector3d());
        rocket.getMetadata().put("attached_to_target", OptionalInt.of(PLAYER_ID));
        world.getEntities().put(FIREWORK_ID, rocket);
    }

    private static void assertPhysicsResultBits(PhysicsResult expected, PhysicsResult actual, long sequence) {
        assertVectorBits(expected.position(), actual.position(), "position", sequence);
        assertVectorBits(expected.velocity(), actual.velocity(), "velocity", sequence);
        assertEquals(expected.collidedX(), actual.collidedX(), "collidedX frame " + sequence);
        assertEquals(expected.collidedZ(), actual.collidedZ(), "collidedZ frame " + sequence);
        assertEquals(expected.onGround(), actual.onGround(), "onGround frame " + sequence);
        assertRawBits(expected.fallDistance(), actual.fallDistance(), "fallDistance", sequence);
        assertEquals(expected.pose(), actual.pose(), "pose frame " + sequence);
    }

    private static void assertVectorBits(Vector3d expected, Vector3d actual, String name, long sequence) {
        assertRawBits(expected.x, actual.x, name + ".x", sequence);
        assertRawBits(expected.y, actual.y, name + ".y", sequence);
        assertRawBits(expected.z, actual.z, name + ".z", sequence);
    }

    private static void assertRawBits(double expected, double actual, String name, long sequence) {
        assertEquals(Double.doubleToRawLongBits(expected), Double.doubleToRawLongBits(actual),
                name + " raw bits frame " + sequence);
    }

    private enum Trigger { GLIDING_COLLIDED_X, AUTHORITATIVE_GLIDE_LOSS }

    private record Incident(int schema, int registryStateCount, long continuity, Trigger trigger,
            long triggerSequence, long triggerPhysicsTick, long triggerContinuity,
            int rawFlags, int effectiveFlags, boolean glidingBefore, boolean glidingAfter,
            List<Frame> frames) {
        static Incident read(byte[] bytes) throws IOException {
            try (DataInputStream input = new DataInputStream(new BufferedInputStream(
                    new java.io.ByteArrayInputStream(bytes)))) {
                require(input.readInt() == MAGIC, "invalid magic");
                require(input.readInt() == VERSION, "unsupported container version");
                int schema = input.readInt();
                require(schema == VERSION, "unsupported schema version");
                int registryStateCount = input.readInt();
                require(input.readNBytes(32).length == 32, "truncated collision digest");
                require(input.readNBytes(32).length == 32, "truncated physics-resource digest");
                long continuity = input.readLong();
                int triggerOrdinal = input.readUnsignedByte();
                require(triggerOrdinal < Trigger.values().length, "invalid trigger");
                Trigger trigger = Trigger.values()[triggerOrdinal];
                long triggerSequence = input.readLong();
                long triggerPhysicsTick = input.readLong();
                long triggerContinuity = input.readLong();
                int rawFlags = input.readInt();
                int effectiveFlags = input.readInt();
                boolean glidingBefore = input.readBoolean();
                boolean glidingAfter = input.readBoolean();
                input.readLong(); // capturedAtMillis is provenance only.
                int count = input.readInt();
                require(count >= 0 && count <= 64, "invalid frame count");
                List<Frame> frames = new ArrayList<>(count);
                for (int i = 0; i < count; i++) frames.add(Frame.read(input));
                require(input.read() == -1, "trailing fixture bytes");
                return new Incident(schema, registryStateCount, continuity, trigger,
                        triggerSequence, triggerPhysicsTick, triggerContinuity,
                        rawFlags, effectiveFlags, glidingBefore, glidingAfter, List.copyOf(frames));
            }
        }
    }

    private record Frame(long sequence, long physicsTick, long continuity, PhysicsState state,
            PlayerInput input, float yaw, float pitch, MovementParameters parameters,
            double halfWidth, double height, int minWorldY, int maxWorldY,
            boolean gliding, boolean attached, boolean jumpAllowed, PhysicsResult expected,
            boolean transcriptOverflow, long droppedEvents, long[] transcript) {
        static Frame read(DataInputStream input) throws IOException {
            long sequence = input.readLong();
            long physicsTick = input.readLong();
            long continuity = input.readLong();
            Vector3d position = vector(input);
            Vector3d velocity = vector(input);
            boolean onGround = input.readBoolean();
            double fallDistance = value(input.readLong());
            PlayerPose pose = pose(input.readUnsignedByte());
            PlayerInput playerInput = playerInput(input.readUnsignedByte());
            float yaw = Float.intBitsToFloat(input.readInt());
            float pitch = Float.intBitsToFloat(input.readInt());
            MovementParameters parameters = new MovementParameters(
                    value(input.readLong()), value(input.readLong()), value(input.readLong()),
                    value(input.readLong()), value(input.readLong()), value(input.readLong()),
                    value(input.readLong()), input.readBoolean(), input.readInt(), input.readBoolean(),
                    value(input.readLong()), value(input.readLong()));
            double halfWidth = value(input.readLong());
            double height = value(input.readLong());
            int minWorldY = input.readInt();
            int maxWorldY = input.readInt();
            boolean gliding = input.readBoolean();
            boolean attached = input.readBoolean();
            boolean jumpAllowed = input.readBoolean();
            Vector3d resultPosition = vector(input);
            Vector3d resultVelocity = vector(input);
            boolean collidedX = input.readBoolean();
            boolean collidedZ = input.readBoolean();
            boolean resultOnGround = input.readBoolean();
            double resultFallDistance = value(input.readLong());
            PlayerPose resultPose = pose(input.readUnsignedByte());
            boolean overflow = input.readBoolean();
            long dropped = input.readLong();
            int length = input.readInt();
            require(length >= 0 && length <= 65_536, "invalid transcript length");
            long[] transcript = new long[length];
            for (int i = 0; i < length; i++) transcript[i] = input.readLong();
            return new Frame(sequence, physicsTick, continuity,
                    new PhysicsState(position, velocity, onGround, fallDistance, pose),
                    playerInput, yaw, pitch, parameters, halfWidth, height, minWorldY, maxWorldY,
                    gliding, attached, jumpAllowed,
                    new PhysicsResult(resultPosition, resultVelocity, collidedX, collidedZ,
                            resultOnGround, resultFallDistance, resultPose),
                    overflow, dropped, transcript);
        }

        private static Vector3d vector(DataInputStream input) throws IOException {
            return new Vector3d(value(input.readLong()), value(input.readLong()), value(input.readLong()));
        }

        private static PlayerPose pose(int ordinal) throws IOException {
            require(ordinal < PlayerPose.values().length, "invalid pose");
            return PlayerPose.values()[ordinal];
        }

        private static PlayerInput playerInput(int flags) {
            return new PlayerInput(flag(flags, 0), flag(flags, 1), flag(flags, 2), flag(flags, 3),
                    flag(flags, 4), flag(flags, 5), flag(flags, 6));
        }

        private static boolean flag(int flags, int bit) { return (flags & (1 << bit)) != 0; }
    }

    /** Immutable terrain/chunk observations reconstructed only from one frame's transcript. */
    private static final class TranscriptWorld extends World {
        private static final int BLOCK_STATE = 1;
        private static final int QUERY_BEGIN = 2;
        private static final int CANDIDATE = 3;
        private static final int SHAPE = 4;
        private static final int EARLY_STOP = 5;
        private static final int SUPPORT_SELECTION = 6;
        private static final int QUERY_END = 7;
        private static final int CHUNK_LOADED = 8;

        private final Map<BlockPosition, Integer> blocks;
        private final Map<ChunkPosition, Boolean> chunks;

        private TranscriptWorld(Map<BlockPosition, Integer> blocks, Map<ChunkPosition, Boolean> chunks) {
            this.blocks = Map.copyOf(blocks);
            this.chunks = Map.copyOf(chunks);
        }

        static TranscriptWorld from(Frame frame, int stateCount) throws IOException {
            return from(List.of(frame), stateCount);
        }

        static TranscriptWorld from(List<Frame> frames, int stateCount) throws IOException {
            Map<BlockPosition, Integer> blocks = new HashMap<>();
            Map<ChunkPosition, Boolean> chunks = new HashMap<>();
            for (Frame frame : frames) parse(frame, stateCount, blocks, chunks);
            return new TranscriptWorld(blocks, chunks);
        }

        private static void parse(Frame frame, int stateCount,
                Map<BlockPosition, Integer> blocks,
                Map<ChunkPosition, Boolean> chunks) throws IOException {
            long[] words = frame.transcript();
            int cursor = 0;
            int openQueries = 0;
            while (cursor < words.length) {
                int event = exactInt(words[cursor++], "event");
                switch (event) {
                    case BLOCK_STATE, CANDIDATE -> {
                        requireRemaining(words, cursor, 4, event);
                        BlockPosition position = new BlockPosition(exactInt(words[cursor], "block x"),
                                exactInt(words[cursor + 1], "block y"),
                                exactInt(words[cursor + 2], "block z"));
                        int state = exactInt(words[cursor + 3], "state id");
                        require(state >= 0 && state < stateCount, "invalid state id");
                        merge(blocks, position, state, "block state");
                        cursor += 4;
                    }
                    case QUERY_BEGIN -> {
                        requireRemaining(words, cursor, 7, event);
                        openQueries++;
                        cursor += 7;
                    }
                    case SHAPE -> {
                        requireRemaining(words, cursor, 9, event);
                        cursor += 9;
                    }
                    case EARLY_STOP -> {
                        requireRemaining(words, cursor, 1, event);
                        cursor++;
                    }
                    case SUPPORT_SELECTION -> {
                        requireRemaining(words, cursor, 6, event);
                        BlockPosition position = new BlockPosition(exactInt(words[cursor], "support x"),
                                exactInt(words[cursor + 1], "support y"),
                                exactInt(words[cursor + 2], "support z"));
                        int state = exactInt(words[cursor + 3], "support state");
                        require(state >= 0 && state < stateCount, "invalid support state id");
                        merge(blocks, position, state, "support state");
                        cursor += 6;
                    }
                    case QUERY_END -> {
                        requireRemaining(words, cursor, 2, event);
                        require(openQueries > 0, "query end without query begin");
                        openQueries--;
                        cursor += 2;
                    }
                    case CHUNK_LOADED -> {
                        requireRemaining(words, cursor, 4, event);
                        cursor++; // Query source is irrelevant to the immutable observation.
                        ChunkPosition position = new ChunkPosition(exactInt(words[cursor], "chunk x"),
                                exactInt(words[cursor + 1], "chunk z"));
                        long loadedWord = words[cursor + 2];
                        require(loadedWord == 0 || loadedWord == 1, "invalid chunk-loaded value");
                        merge(chunks, position, loadedWord != 0, "chunk state");
                        cursor += 3;
                    }
                    default -> throw new IOException("unknown transcript event " + event);
                }
            }
            require(openQueries == 0, "unterminated transcript query");
            require(frame.minWorldY() <= frame.maxWorldY(), "invalid world bounds");
        }

        @Override public boolean chunkLoaded(int chunkX, int chunkZ) {
            Boolean loaded = chunks.get(new ChunkPosition(chunkX, chunkZ));
            if (loaded == null) throw new AssertionError("unrecorded chunk read " + chunkX + "," + chunkZ);
            return loaded;
        }

        @Override public int getBlockAt(Vector3d position) {
            return stateAt(position);
        }

        @Override public BlockState getBlockStateAt(Vector3d position) {
            return BlockStateParser.Instance.parseStateId(stateAt(position));
        }

        private int stateAt(Vector3d position) {
            BlockPosition key = new BlockPosition((int) Math.floor(position.x),
                    (int) Math.floor(position.y), (int) Math.floor(position.z));
            Integer state = blocks.get(key);
            if (state == null) throw new AssertionError("unrecorded block read " + key);
            return state;
        }

        private record BlockPosition(int x, int y, int z) {}
        private record ChunkPosition(int x, int z) {}
    }

    private static void requireRemaining(long[] words, int cursor, int count, int event) throws IOException {
        require(cursor <= words.length - count, "truncated transcript event " + event);
    }

    private static int exactInt(long value, String label) throws IOException {
        require((int) value == value, label + " outside int range");
        return (int) value;
    }

    private static <K, V> void merge(Map<K, V> values, K key, V value, String label) throws IOException {
        V previous = values.putIfAbsent(key, value);
        require(previous == null || previous.equals(value), "conflicting " + label + " for " + key);
    }

    private static double value(long bits) { return Double.longBitsToDouble(bits); }

    private static void require(boolean condition, String message) throws IOException {
        if (!condition) throw new IOException(message);
    }
}
