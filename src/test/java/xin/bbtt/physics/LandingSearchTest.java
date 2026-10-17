package xin.bbtt.physics;

import java.util.HashSet;
import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.movements.LandingMemory;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class LandingSearchTest {
    private static final class Terrain extends World {
        boolean loaded = true;
        double start = 20;
        int columns;
        double farthest;
        @Override public boolean chunkLoaded(int x, int z) { return loaded; }
        @Override public Set<ChunkPosition> loadedChunks() {
            Set<ChunkPosition> chunks = new HashSet<>();
            if (loaded) for (int x = -10; x <= 10; x++) for (int z = -10; z <= 10; z++)
                chunks.add(new ChunkPosition(x, z));
            return chunks;
        }
        @Override public OptionalDouble findHighestCollisionTopExcluding(double minX, double minY,
                double minZ, double maxX, double maxY, double maxZ, Set<Vector3i> excluded) {
            columns++;
            farthest = Math.max(farthest, Math.hypot((minX + maxX) / 2, (minZ + maxZ) / 2));
            return loaded && minZ >= start && minY <= 0 && maxY >= 0
                    ? OptionalDouble.of(0) : OptionalDouble.empty();
        }
    }
    private static PhysicsState state(double y) {
        return new PhysicsState(new Vector3d(.5, y, .5), new Vector3d(0, -.1, .8), false);
    }
    @Test void stopsBeforeScanningFartherChunks() {
        Terrain world = new Terrain();
        var selected = ElytraFlightController.selectForcedLandingTarget(world, state(100), 0, false);
        assertEquals(new Vector3d(.5, 0, 24.5), selected);
        assertTrue(world.farthest < 32, "no terrain query in distant chunk rings after finding reachable ground");
    }
    @Test void boundedVoidScanContinuesNextTickUntilDistantIsland() {
        Terrain world = new Terrain();
        world.start = 138;
        LandingSearch search = new LandingSearch();
        Vector3d result = null;
        int ticks = 0;
        while (result == null && ticks++ < 100) {
            world.columns = 0;
            result = search.select(world, state(100), new int[0], false, MovementParameters.DEFAULT, LandingSearch.BUDGET);
            // Each sample has one top query plus at most 25 support queries;
            // one additional footprint check is for the point directly below.
            assertTrue(world.columns <= (LandingSearch.BUDGET + 1) * 26);
        }
        assertTrue(ticks > 1);
        assertNotNull(result);
        assertTrue(result.z > 128);
    }
    @Test void unloadedMemorySteersUnpoweredUsingLongestCandidateGlide() {
        Terrain world = new Terrain();
        LandingMemory memory = new LandingMemory();
        memory.observe(world, new Vector3d(.5, 0, 60.5));
        Vector3d target = memory.nearest(state(100).position());
        assertNotNull(target);
        world.loaded = false;
        var initial = state(100);
        var decision = new ElytraFlightController().forceLand(world, initial, 180, 10,
                false, new int[0], 16, memory, new LandingSearch());
        assertFalse(decision.firework());
        assertEquals(ElytraFlightController.Phase.FORCED_LANDING, decision.phase());
        float heading = (float) Math.toDegrees(Math.atan2(initial.position().x - target.x, target.z - initial.position().z));
        var range = new GlideRange(initial, MovementParameters.DEFAULT, ElytraFlightController.landingPitches());
        double selected = range.projectedRange(target.y, decision.yaw(), decision.pitch())
                * Math.cos(Math.toRadians(decision.yaw() - heading));
        for (float offset : new float[]{0, -30, 30, -60, 60, -100, 100, 180})
            for (float pitch : ElytraFlightController.landingPitches())
                assertTrue(selected + 1e-7 >= range.projectedRange(target.y, heading + offset, pitch)
                        * Math.cos(Math.toRadians(offset)));
        assertTrue(Math.cos(Math.toRadians(decision.yaw() - heading)) > 0);
    }
    @Test void memoryInfoIsEmittedOnlyOnTargetTransitions() {
        var logger = (ch.qos.logback.classic.Logger) xin.bbtt.MovementSync.getLogger();
        var oldLevel = logger.getLevel();
        var messages = new ch.qos.logback.core.read.ListAppender<ch.qos.logback.classic.spi.ILoggingEvent>();
        messages.start();
        logger.addAppender(messages);
        logger.setLevel(ch.qos.logback.classic.Level.INFO);
        try {
            var search = new LandingSearch();
            search.selectedMemory(new Vector3d(1, 0, 2));
            search.selectedMemory(new Vector3d(1, 0, 2));
            search.selectedMemory(null);
            search.selectedMemory(new Vector3d(1, 0, 2));
            assertEquals(2, messages.list.size());
            assertTrue(messages.list.stream().allMatch(e -> e.getLevel() == ch.qos.logback.classic.Level.INFO
                    && e.getFormattedMessage().contains("remembered ground")));
        } finally {
            logger.detachAppender(messages);
            logger.setLevel(oldLevel);
            messages.stop();
        }
    }
    @Test void rememberedTargetWithRocketTurnsTowardLandInUnknownTerrain() {
        Terrain world = new Terrain();
        LandingMemory memory = new LandingMemory();
        memory.observe(world, new Vector3d(.5, 0, 150.5));
        world.loaded = false;
        var decision = new ElytraFlightController().forceLand(world, state(3), 180, 0,
                false, new int[]{1}, 16, memory, new LandingSearch());
        assertTrue(decision.firework());
        assertTrue(Math.cos(Math.toRadians(decision.yaw())) > 0);
    }
    // Previous implementation retained only as a performance reference: eagerly
    // allocate/sort every loaded sample before examining the first terrain column.
    private Vector3d legacySelect(Terrain world, PhysicsState state) {
        var samples = new java.util.ArrayList<Vector3d>();
        for (var c : world.loadedChunks())
            for (int x = c.x() * 16; x < c.x() * 16 + 16; x += 4)
                for (int z = c.z() * 16; z < c.z() * 16 + 16; z += 4)
                    samples.add(new Vector3d(x + .5, 0, z + .5));
        samples.sort(java.util.Comparator.comparingDouble(s -> Math.hypot(s.x - .5, s.z - .5)));
        var range = new GlideRange(state, MovementParameters.DEFAULT, ElytraFlightController.landingPitches());
        var ranges = new java.util.HashMap<Double, Double>();
        for (var sample : samples) {
            var site = ElytraFlightController.landingSite(world, sample.x, sample.z, World.getMaxWorldY());
            if (site != null && Math.hypot(site.x - .5, site.z - .5)
                    <= ranges.computeIfAbsent(site.y, y -> range.atHeight(y, 0, false))) return site;
        }
        throw new AssertionError("benchmark has reachable land");
    }
    private Object legacyDecision(Terrain world, ElytraFlightController controller,
            java.lang.reflect.Method emergency) throws Exception {
        var initial = state(100);
        var target = legacySelect(world, initial);
        boolean boost = Math.hypot(target.x - .5, target.z - .5)
                > new GlideRange(initial, MovementParameters.DEFAULT, ElytraFlightController.landingPitches()).atHeight(target.y, 0, false);
        float heading = (float) Math.toDegrees(Math.atan2(.5 - target.x, target.z - .5));
        return emergency.invoke(controller, world, initial, 0f, 0f, false, target, 16d, heading, boost, false, false);
    }
    @Test void reportsWarmEmergencyLatency() throws Exception {
        Terrain world = new Terrain();
        var controller = new ElytraFlightController();
        var search = new LandingSearch();
        for (int i = 0; i < 100; i++) controller.forceLand(world, state(100), 0, 0, false, new int[0], 16, null, search);
        long start = System.nanoTime();
        for (int i = 0; i < 200; i++) controller.forceLand(world, state(100), 0, 0, false, new int[0], 16, null, search);
        double current = (System.nanoTime() - start) / 200_000_000.0;
        var emergency = ElytraFlightController.class.getDeclaredMethod("emergencyDecision", World.class,
                PhysicsState.class, float.class, float.class, boolean.class, Vector3d.class,
                double.class, float.class, boolean.class, boolean.class, boolean.class);
        emergency.setAccessible(true);
        for (int i = 0; i < 100; i++) legacyDecision(world, controller, emergency);
        start = System.nanoTime();
        for (int i = 0; i < 200; i++) legacyDecision(world, controller, emergency);
        System.err.printf("Emergency decision warm mean: incremental %.3f ms, eager-sort reference %.3f ms%n",
                current, (System.nanoTime() - start) / 200_000_000.0);
    }
}
