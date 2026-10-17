package xin.bbtt.flight;

import java.util.List;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsStepper;

/** Test-only extension: its predictor and executor need no vanilla glide controls. */
final class LevelFlightModeExample implements FlightMode {
    record Level(double speed) implements FlightAction {}
    enum Phase implements FlightPhase { LEVEL }

    final PhysicsStepper stepper = (world, state, width, height) ->
            new PhysicsResult(state.position().add(0, 0, 1),
                    new org.joml.Vector3d(0, 0, 1), false, false, false, 0);
    int executions;
    int takeoffTicks;
    int landingTicks;
    boolean landingEnabled;
    PhysicsStepper transitionStepper = stepper;
    FlightAction lastExecuted;

    @Override public List<? extends FlightAction> candidates(FlightContext context) {
        return List.of(new Level(1));
    }

    @Override public Trajectory predict(FlightContext context, FlightAction action) {
        return new Trajectory(context.state(), List.of(stepper.step(context.world(), context.state(), .299, .6)));
    }

    @Override public void execute(FlightContext context, FlightAction action, MovementSync sync) {
        if (!(action instanceof Level)) throw new IllegalArgumentException("Not a level action");
        executions++;
        lastExecuted = action;
        if (sync != null) {
            var next = stepper.step(context.world(), context.state(), .299, .6);
            sync.position.set(next.position());
            sync.velocity.set(next.velocity());
            sync.onGround.set(next.onGround());
            sync.fallDistance.set(next.fallDistance());
        }
    }

    @Override public FlightPhase phase(FlightContext context, FlightAction action) { return Phase.LEVEL; }

    @Override public Trajectory previewTakeoff(FlightContext context) {
        return new Trajectory(context.state(), List.of(transitionStepper.step(context.world(), context.state(), .299, .6)), new Level(1));
    }

    @Override public Trajectory previewLanding(FlightContext context) {
        if (!landingEnabled) return null;
        return new Trajectory(context.state(), List.of(new PhysicsResult(context.state().position(),
                new org.joml.Vector3d(), false, false, true, 0)), new Level(1));
    }

    @Override public Movement createLanding(FlightContext context) {
        if (!landingEnabled) return null;
        return new Movement() {
            @Override public void init() { finished = false; }
            @Override public void onTick() { landingTicks++; finished = true; }
            @Override public long getTime() { return -1; }
            @Override public void onStop() {}
        };
    }

    @Override public Movement createTakeoff(FlightContext context) {
        return new Movement() {
            @Override public void init() { finished = false; }
            @Override public void onTick() { takeoffTicks++; finished = true; }
            @Override public long getTime() { return -1; }
            @Override public void onStop() {}
        };
    }
}
