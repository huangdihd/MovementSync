package xin.bbtt.flight;

import java.util.List;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.PublicFlightBridge;

/**
 * Built-in vanilla glide, using the same predictor as the legacy controller.
 * Prediction methods are pure queries on the navigation thread. Execution requires
 * the navigation guard and leaves equipment, lifecycle and rocket cooldown to its caller.
 */
public final class VanillaGlideMode implements FlightMode {
    /** Stateless shared built-in mode, reusable across navigation requests. */
    public static final VanillaGlideMode INSTANCE = new VanillaGlideMode();
    /** Retains the vanilla lifecycle's exact gliding-state termination condition. Pure observation. */
    @Override public boolean isAirborne(FlightContext context, MovementSync sync) { return sync.isGliding(); }
    /** Returns the stable built-in diagnostic name. */
    @Override public String name() { return "VANILLA_GLIDE"; }
    /** Enumerates non-null vanilla controls without changing live physics. */
    @Override public List<FlightAction.Glide> candidates(FlightContext c) { return PublicFlightBridge.candidates(c); }
    /** Previews a vanilla Glide action from the supplied state; never returns null. */
    @Override public Trajectory predict(FlightContext c, FlightAction action) {
        return PublicFlightBridge.predict(c, (FlightAction.Glide) action);
    }
    /** Returns vanilla pull-up previews beginning at the main trajectory endpoint. */
    @Override public List<Trajectory> recover(FlightContext c, FlightAction action, Trajectory main) {
        return PublicFlightBridge.recover(c, (FlightAction.Glide) action, main);
    }
    /** Pure physics preview; equipment changes and deployment are deferred to execution. */
    @Override public Trajectory previewTakeoff(FlightContext c) {
        return xin.bbtt.movements.VanillaTransitionBridge.preview(c, true);
    }
    @Override public Trajectory previewLanding(FlightContext c) {
        return xin.bbtt.movements.VanillaTransitionBridge.preview(c, false);
    }
    @Override public xin.bbtt.movement.Movement createTakeoff(FlightContext c) {
        return xin.bbtt.movements.VanillaTransitionBridge.takeoff();
    }
    @Override public xin.bbtt.movement.Movement createLanding(FlightContext c) {
        return xin.bbtt.movements.VanillaTransitionBridge.landing(c);
    }
    /** Applies orientation and permitted rocket use; caller holds navigation lock before sync monitor. */
    @Override public void execute(FlightContext c, FlightAction action, MovementSync sync) {
        var a = (FlightAction.Glide) action;
        sync.setYawToward(a.yaw());
        sync.pitch.set(a.pitch());
        if (a.firework() && !c.attached() && c.fireworks() > 0 && !sync.hasAttachedFirework()) sync.useFirework();
    }
}
