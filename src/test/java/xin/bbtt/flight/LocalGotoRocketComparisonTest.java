package xin.bbtt.flight;

import java.util.Collections;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.physics.*;
import static org.junit.jupiter.api.Assertions.*;

class LocalGotoRocketComparisonTest {
    private static LocalLookaheadSolver.PrefixResult unpowered(int ticks, double score) {
        var state = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(), false);
        var control = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);
        return new LocalLookaheadSolver.PrefixResult(control,
                Collections.nCopies(ticks, new FlightControlAdmission.Step(control, state)),
                ticks, 0, ticks == 0 ? LocalLookaheadSolver.PrefixStatus.UNPROVEN
                        : LocalLookaheadSolver.PrefixStatus.SAFE_PREFIX,
                state, score, new LocalLookaheadSolver.Rejections(0, 0, 0, 0));
    }

    @Test void safeHorizonTakesPriorityOverProgressForRocketComparison() {
        assertTrue(LocalGotoController.preferPowered(12, 1, unpowered(3, 10)));
        assertFalse(LocalGotoController.preferPowered(3, 10, unpowered(12, 1)));
        assertTrue(LocalGotoController.preferPowered(12, 2, unpowered(12, 1)));
        assertFalse(LocalGotoController.preferPowered(12, 1, unpowered(12, 1)));
    }

    @Test void rocketAlwaysNeedsAnExecutableUnpoweredBranchOfTheSameControl() {
        assertFalse(LocalGotoController.preferPowered(0, 100, unpowered(0, -100)));
        assertTrue(LocalGotoController.preferPowered(1, 0, unpowered(0, 100)));
    }
}
