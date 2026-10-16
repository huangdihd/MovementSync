package xin.bbtt.physics;

import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

/** Golden decisions recorded before introducing flight targets. */
final class FixedFlightCharacterizationTest {
    private static final class Terrain extends World {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public boolean isBoxColliding(double x, double y, double z,
                double X, double Y, double Z) { return y < 0; }
        @Override public OptionalDouble findHighestCollisionTopExcluding(double x, double y, double z,
                double X, double Y, double Z, Set<Vector3i> excluded) {
            return y <= 0 && Y >= 0 ? OptionalDouble.of(0) : OptionalDouble.empty();
        }
    }

    @Test void recordedDecisions() {
        String[] golden = {
                "1135869952,-1034158080,false,DESCENT",
                "1135869952,-1034158080,false,DESCENT",
                "1135869952,1092616192,true,CRUISE",
                "1135869952,1092616192,false,CRUISE",
                "1135869952,1092616192,true,CRUISE",
                "1135869952,1092616192,false,CRUISE",
                "1135869952,-1034158080,false,DESCENT",
                "1135869952,-1034158080,false,DESCENT",
                "1135869952,1092616192,false,DESCENT",
                "1135869952,1092616192,false,DESCENT",
                "1135869952,1092616192,true,CRUISE",
                "1135869952,1092616192,false,CRUISE"
        };
        int sample = 0;
        var controller = new ElytraFlightController();
        for (double height : new double[]{20, 80}) {
            for (double distance : new double[]{8, 120, 800}) {
                for (boolean attached : new boolean[]{false, true}) {
                    var decision = controller.decide(new Terrain(), new PhysicsState(
                            new Vector3d(0, height, 0), new Vector3d(0, -0.1, 0.3), false),
                            370, -10, attached, new Vector3d(0, 0, distance), 5, 3);
                    org.junit.jupiter.api.Assertions.assertEquals(golden[sample++],
                            Float.floatToRawIntBits(decision.yaw()) + ","
                                    + Float.floatToRawIntBits(decision.pitch()) + "," + decision.firework()
                                    + "," + decision.phase());
                }
            }
        }
    }
}
