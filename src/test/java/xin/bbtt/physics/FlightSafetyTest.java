package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.physics.FlightSafety.EmergencyCost;
import xin.bbtt.physics.FlightSafety.Impact;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class FlightSafetyTest {
    @Test void damageClassesPreserveRawDamageAndLethalBoundary() {
        assertEquals(0, Impact.SAFE.rank(20));
        assertEquals(1, new Impact(false, 5, 0).rank(20));
        assertEquals(1, new Impact(false, 5, 5).rank(6));
        assertEquals(2, new Impact(false, 5, 5).rank(5));
        assertEquals(3, Impact.UNKNOWN.rank(20));
    }

    @Test void emergencyOrderingIsLexicographic() {
        assertTrue(new EmergencyCost(0, 100, 2, 100).compareTo(new EmergencyCost(1, 0, 0, 0)) < 0);
        assertTrue(new EmergencyCost(1, 1, 2, 100).compareTo(new EmergencyCost(1, 2, 0, 0)) < 0);
        assertTrue(new EmergencyCost(1, 1, 0, 100).compareTo(new EmergencyCost(1, 1, 1, 0)) < 0);
        assertTrue(new EmergencyCost(1, 1, 0, 1).compareTo(new EmergencyCost(1, 1, 0, 2)) < 0);
        assertEquals(0, new EmergencyCost(1, 1, 0, 1).compareTo(new EmergencyCost(1, 1, 0, 1)));
    }

    @Test void unknownTerrainDoesNotBecomeNumericDamage() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return false; }
        };
        FlightSafety safety = new FlightSafety(MovementParameters.DEFAULT, DamageParameters.NONE, 20);
        PhysicsState state = new PhysicsState(new Vector3d(0, 40, 0), new Vector3d(0, 0, .8), false);
        assertEquals(Impact.UNKNOWN, safety.predictEmergencyImpact(world, state, 0, 0, false));
        assertEquals(Double.POSITIVE_INFINITY, safety.emergencyDamage(world, state, 0, 0, false));
    }
}
