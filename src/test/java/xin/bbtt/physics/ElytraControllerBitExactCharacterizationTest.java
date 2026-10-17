package xin.bbtt.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.function.Supplier;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.movements.LandingMemory;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Frozen outputs from the pre-refactor controller, including IEEE-754 sign bits. */
final class ElytraControllerBitExactCharacterizationTest {
    private static final class Terrain extends World {
        boolean loaded = true;
        boolean solid = true;
        boolean wall;
        boolean trees;
        double start = -1000;
        @Override public boolean chunkLoaded(int x, int z) { return loaded; }
        @Override public Set<ChunkPosition> loadedChunks() {
            Set<ChunkPosition> chunks = new java.util.HashSet<>();
            if (loaded) for (int x = -3; x <= 3; x++) for (int z = -3; z <= 3; z++)
                chunks.add(new ChunkPosition(x, z));
            return chunks;
        }
        private double top(double x, double z) { return trees && Math.abs(x) < 3 && z > 7 && z < 15 ? 8 : 0; }
        @Override public boolean isBoxColliding(double x, double y, double z, double xx, double yy, double zz) {
            return solid && zz >= start && y < top((x + xx) / 2, (z + zz) / 2)
                    || wall && x < 2 && xx > -2 && zz > 8 && z < 9 && y < 30;
        }
        @Override public OptionalDouble findHighestCollisionTopExcluding(double x, double y, double z,
                double xx, double yy, double zz, Set<Vector3i> excluded) {
            double top = top((x + xx) / 2, (z + zz) / 2);
            return solid && z >= start && y <= top && yy >= top ? OptionalDouble.of(top) : OptionalDouble.empty();
        }
    }
    private static PhysicsState state(double height) {
        return new PhysicsState(new Vector3d(.5, height, .5), new Vector3d(0, -.4, .8), false);
    }
    private static String bits(double value) { return Long.toHexString(Double.doubleToRawLongBits(value)); }
    private static String result(Supplier<?> operation) {
        try {
            Object value = operation.get();
            if (value instanceof ElytraFlightController.Decision d)
                return Integer.toHexString(Float.floatToRawIntBits(d.yaw())) + ","
                        + Integer.toHexString(Float.floatToRawIntBits(d.pitch())) + "," + d.firework() + "," + d.phase();
            if (value instanceof Vector3d v) return bits(v.x) + "," + bits(v.y) + "," + bits(v.z);
            return String.valueOf(value);
        } catch (RuntimeException failure) {
            return failure.getClass().getSimpleName() + ":" + failure.getMessage();
        }
    }
    private static void record(List<String> rows, String label, Supplier<?> operation) {
        rows.add(label + "=" + result(operation));
    }
    @Test void frozenControllerOutputs() throws Exception {
        List<String> rows = new ArrayList<>();
        for (String terrain : List.of("flat", "wall", "trees", "void", "unknown")) {
            Terrain world = new Terrain();
            world.wall = terrain.equals("wall"); world.trees = terrain.equals("trees");
            world.solid = !terrain.equals("void"); world.loaded = !terrain.equals("unknown");
            for (double height : new double[]{.6, 3, 20, 80}) for (boolean attached : new boolean[]{false, true})
                for (int rockets : new int[]{0, 2}) {
                    String key = terrain + "/" + height + "/" + attached + "/" + rockets;
                    PhysicsState state = state(height);
                    for (String kind : List.of("fixedNear", "fixedFar", "entity", "lostEntity")) {
                        Vector3d target = new Vector3d(.5, 0, kind.equals("fixedFar") ? 800 : 24.5);
                        FlightTarget destination;
                        if (kind.contains("Entity") || kind.equals("entity")) {
                            EntityFlightTarget entity = new EntityFlightTarget(42, 8, target);
                            if (kind.equals("lostEntity")) entity.sample(world);
                            destination = entity;
                        } else destination = new FixedFlightTarget(target);
                        record(rows, key + "/" + kind, () -> new ElytraFlightController().decide(
                                world, state, 397, 10, attached, destination, rockets, 3));
                    }
                    record(rows, key + "/force", () -> new ElytraFlightController().forceLand(world, state, 397, 10, attached, rockets, 3));
                    record(rows, key + "/memory", () -> {
                        LandingMemory memory = new LandingMemory();
                        memory.takeoff(new Terrain(), new Vector3d(.5, 0, .5));
                        return new ElytraFlightController().forceLand(world, state, 397, 10, attached, rockets, 3, memory);
                    });
                    record(rows, key + "/search", () -> new ElytraFlightController().forceLand(world, state, 397, 10,
                            attached, rockets == 0 ? new int[0] : new int[]{1, 3}, 3, null, new LandingSearch()));
                    record(rows, key + "/select", () -> ElytraFlightController.selectForcedLandingTarget(world, state, rockets, attached));
                    record(rows, key + "/hold", () -> new ElytraFlightController().emergencyHold(world, state, 397, 80, attached));
                }
        }
        Terrain world = new Terrain();
        ElytraFlightController controller = new ElytraFlightController();
        PhysicsState ground = new PhysicsState(new Vector3d(.5, 0, .5), new Vector3d(), true);
        record(rows, "ground/decide", () -> controller.decide(world, ground, -720, 80, false, new Vector3d(), 0, 3));
        record(rows, "ground/force", () -> controller.forceLand(world, ground, -720, 80, false, 0, 3));
        for (float yaw : new float[]{Float.NaN, Float.POSITIVE_INFINITY}) {
            record(rows, "invalid/yaw/" + yaw, () -> controller.decide(world, state(20), yaw, 0, false, new Vector3d(), 0, 3));
            record(rows, "invalid/forceYaw/" + yaw, () -> controller.forceLand(world, state(20), yaw, 0, false, 0, 3));
        }
        for (double radius : new double[]{0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            record(rows, "invalid/radius/" + radius, () -> controller.decide(world, state(20), 0, 0, false, new Vector3d(), 0, radius));
            record(rows, "invalid/forceRadius/" + radius, () -> controller.forceLand(world, state(20), 0, 0, false, 0, radius));
        }
        record(rows, "invalid/rockets", () -> controller.decide(world, state(20), 0, 0, false, new Vector3d(), -1, 3));
        record(rows, "invalid/forceRockets", () -> controller.forceLand(world, state(20), 0, 0, false, -1, 3));
        record(rows, "invalid/selectRockets", () -> ElytraFlightController.selectForcedLandingTarget(world, state(20), -1, false));
        record(rows, "invalid/target", () -> controller.decide(world, state(20), 0, 0, false, new Vector3d(Double.NaN), 0, 3));
        record(rows, "invalid/world", () -> controller.decide(null, state(20), 0, 0, false, new Vector3d(), 0, 3));
        Terrain island = new Terrain(); island.start = 40;
        for (boolean attached : new boolean[]{false, true}) for (int rockets : new int[]{0, 1}) {
            record(rows, "island/" + attached + "/" + rockets, () -> controller.forceLand(island,
                    new PhysicsState(new Vector3d(.5, 3, .5), new Vector3d(0, -.1, .5), false),
                    0, 0, attached, rockets, 16));
        }
        try (var stream = getClass().getResourceAsStream("elytra-controller-bits.txt")) {
            assertNotNull(stream, "Frozen pre-refactor characterization fixture");
            List<String> expected = new String(stream.readAllBytes(), java.nio.charset.StandardCharsets.UTF_8)
                    .lines().toList();
            assertEquals(743, rows.size());
            assertIterableEquals(expected, rows);
        }
    }
}
