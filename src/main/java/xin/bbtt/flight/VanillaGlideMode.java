package xin.bbtt.flight;

import java.util.List;
import xin.bbtt.MovementSync;
import xin.bbtt.physics.PublicFlightBridge;

/** Standard glide mode; standalone execution leaves lifecycle/cooldown ownership to its caller. */
public final class VanillaGlideMode implements FlightMode {
    public static final VanillaGlideMode INSTANCE = new VanillaGlideMode();
    @Override public String name() { return "VANILLA_GLIDE"; }
    @Override public List<FlightAction.Glide> candidates(FlightContext c) { return PublicFlightBridge.candidates(c); }
    @Override public Trajectory predict(FlightContext c, FlightAction action) {
        return PublicFlightBridge.predict(c, (FlightAction.Glide) action);
    }
    @Override public List<Trajectory> recover(FlightContext c, FlightAction action, Trajectory main) {
        return PublicFlightBridge.recover(c, (FlightAction.Glide) action, main);
    }
    @Override public void execute(FlightContext c, FlightAction action, MovementSync sync) {
        var a = (FlightAction.Glide) action;
        sync.setYawToward(a.yaw());
        sync.pitch.set(a.pitch());
        if (a.firework() && !c.attached() && c.fireworks() > 0 && !sync.hasAttachedFirework()) sync.useFirework();
    }
}
