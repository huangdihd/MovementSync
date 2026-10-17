package xin.bbtt.physics;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.UUID;
import org.geysermc.mcprotocollib.protocol.data.game.entity.type.EntityType;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.Entity.Entity;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

/** Additional pre-refactor goldens; the original 743-row fixture remains frozen. */
final class ElytraControllerSupplementalBitExactTest {
    private static final class FlatWorld extends World {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public boolean isOnGround(Vector3d p) { return p.y == 0; }
        @Override public boolean isBoxColliding(double x, double y, double z, double xx, double yy, double zz) {
            return y < 0;
        }
        @Override public OptionalDouble findHighestCollisionTopExcluding(double x, double y, double z,
                double xx, double yy, double zz, Set<Vector3i> excluded) {
            return y <= 0 && yy >= 0 ? OptionalDouble.of(0) : OptionalDouble.empty();
        }
    }
    private static String bits(double value) { return Long.toHexString(Double.doubleToRawLongBits(value)); }
    private static String vector(Vector3d v) { return bits(v.x) + "," + bits(v.y) + "," + bits(v.z); }
    private static String decision(ElytraFlightController.Decision d) {
        return Integer.toHexString(Float.floatToRawIntBits(d.yaw())) + ","
                + Integer.toHexString(Float.floatToRawIntBits(d.pitch())) + "," + d.firework() + "," + d.phase();
    }
    @Test void frozenLandingAndMovingEntitySnapshots() throws Exception {
        List<String> rows = new ArrayList<>();
        FlatWorld world = new FlatWorld();
        ElytraFlightController controller = new ElytraFlightController();
        int landings = 0;
        for (double height : new double[]{.6, 3, 10}) for (double distance : new double[]{2, 8, 16})
            for (boolean attached : new boolean[]{false, true}) {
                PhysicsState state = new PhysicsState(new Vector3d(.5, height, .5), new Vector3d(0, -.4, .8), false);
                String key = "landing/" + height + "/" + distance + "/" + attached;
                try {
                var d = controller.decide(world, state, 397, 10, attached,
                        new FixedFlightTarget(new Vector3d(.5, 0, .5 + distance)), 2, 3);
                assertEquals(ElytraFlightController.Phase.LANDING, d.phase());
                landings++;
                rows.add(key + "=" + decision(d));
                } catch (IllegalStateException failure) {
                    rows.add(key + "=" + failure.getClass().getSimpleName() + ":" + failure.getMessage());
                }
            }
        assertTrue(landings >= 6, "Several actual LANDING decisions required: " + landings);
        for (String group : List.of("ground", "gliding", "airborne", "turning")) {
            Vector3d initial = new Vector3d(4, group.equals("ground") ? 0 : 18, 12);
            Entity entity = new Entity(99, new UUID(0, 99), EntityType.ARMOR_STAND,
                    0, 0, 0, 0, 0, 0, initial);
            world.getEntities().put(99, entity);
            EntityFlightTarget target = new EntityFlightTarget(99, 3, initial);
            if (group.equals("gliding")) entity.getMetadata().put("shared_flags", (byte) 0x80);
            PhysicsState observer = new PhysicsState(new Vector3d(.5, 20, .5), new Vector3d(.2, -.4, .8), false);
            for (int tick = 1; tick <= 3; tick++) {
                Vector3d next = new Vector3d(initial).add(group.equals("turning") ? new double[]{0, 1, -1, 2}[tick] : tick * .75,
                        group.equals("ground") ? 0 : tick * -.25, tick * 1.5);
                entity.moveTo(next);
                target.sample(world);
                assertFalse(target.lost());
                assertTrue(target.horizontalSpeed() > 0, "An actually moving sampled entity");
                assertEquals(group.equals("ground"), target.landed());
                assertEquals(group.equals("gliding"), target.gliding());
                String key = group + "/" + tick;
                rows.add(key + "/snapshot=" + vector(target.position()) + ";" + vector(target.velocity())
                        + ";" + vector(target.aim(observer)) + ";" + bits(target.horizontalSpeed())
                        + ";" + target.landed() + "," + target.gliding() + "," + target.lost());
                for (boolean attached : new boolean[]{false, true}) for (int rockets : new int[]{0, 2})
                    rows.add(key + "/decision/" + attached + "/" + rockets + "="
                            + decision(controller.decide(world, observer, 397, 10, attached, target, rockets, 3)));
            }
            world.getEntities().remove(99);
            target.sample(world);
            assertTrue(target.lost());
            rows.add(group + "/lost=" + vector(target.position()) + ";" + vector(target.velocity()) + ";"
                    + vector(target.aim(observer)) + ";" + decision(controller.decide(world, observer, 397, 10, false, target, 2, 3)));
        }
        assertEquals(82, rows.size());
        try (var stream = getClass().getResourceAsStream("elytra-controller-supplemental-bits.txt")) {
            assertNotNull(stream, "Frozen supplemental pre-refactor fixture");
            assertIterableEquals(new String(stream.readAllBytes(), StandardCharsets.UTF_8).lines().toList(), rows);
        }
    }
}
