package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class HazardRecoveryWorldIsolationTest {
    @Test void cachedControlsCannotBiasAnotherWorldOnTheSameThread() {
        World first = HazardRecoveryPerformanceTest.sea();
        var state = new PhysicsState(new Vector3d(64, 48, .5), new Vector3d(.8, 0, 0),
                false, 0, PlayerPose.GLIDING);
        int[] rockets = new int[64];
        java.util.Arrays.fill(rockets, 1);
        var resources = new HazardRecovery.Context(rockets, 0, false, null);
        assertEquals(HazardRecovery.Status.RECOVERABLE,
                HazardRecovery.assess(first, state, MovementParameters.DEFAULT, resources).status());
        assertNotNull(HazardRecovery.controlWitness(first));
        assertNull(HazardRecovery.controlWitness(new World()),
                "an unrelated world must start with unbiased candidate ranking");

        assertEquals(HazardRecovery.Status.RECOVERABLE,
                HazardRecovery.assess(first, state, MovementParameters.DEFAULT, resources).status());
        assertNotNull(HazardRecovery.controlWitness(first));
        HazardRecovery.assess(new World(), state, MovementParameters.DEFAULT, resources);
        assertNull(HazardRecovery.controlWitness(),
                "even an unproven assessment must discard another world's hints");
    }
}
