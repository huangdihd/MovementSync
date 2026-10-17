package xin.bbtt.physics;

import java.util.List;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Internal prediction side of a flight mode; no registration surface in A. */
interface FlightMode {
    record Candidate(FlightMode owner, float yaw, float pitch, boolean firework) {}
    List<Candidate> candidates(float yaw, float heading, boolean compareBoost, boolean firework);
    PhysicsStepper stepper(float yaw, float pitch, boolean boosted);
    FlightSafety.RecoveryStrategy recovery(FlightSafety safety);
    FlightTrajectory predict(World world, PhysicsState initial, float yaw, float pitch,
            boolean boosted, int horizon, Vector3d target, boolean emergencyApproach);
}
