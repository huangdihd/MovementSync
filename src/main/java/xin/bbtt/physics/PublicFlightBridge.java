package xin.bbtt.physics;

import java.util.ArrayList;
import java.util.List;
import xin.bbtt.flight.FlightAction;
import xin.bbtt.flight.FlightContext;
import xin.bbtt.flight.Trajectory;
import xin.bbtt.flight.DamageRule;

/** Adapter for the public extension API; the legacy controller remains untouched. */
public final class PublicFlightBridge {
    private PublicFlightBridge() {}
    public static List<FlightAction.Glide> candidates(FlightContext c) {
        double distance = VanillaGlideMode.horizontalDistance(c.state().position(), c.target());
        float heading = distance < .01 ? c.yaw() : (float) Math.toDegrees(Math.atan2(
                c.state().position().x - c.target().x, c.target().z - c.state().position().z));
        double altitude = FlightSelector.routeAltitude(c.world(), c.state().position(), c.target());
        double speed = Math.hypot(c.state().velocity().x, c.state().velocity().z);
        boolean descending = distance <= Math.max(48, (c.state().position().y - c.target().y) * 5 + speed * 24);
        boolean needsBoost = !descending && (c.state().position().y < altitude - 3 || speed < .65);
        var result = new ArrayList<FlightAction.Glide>();
        for (var a : new VanillaGlideMode(c.movement()).candidates(c.yaw(), heading,
                !c.attached() && c.fireworks() > 0 && needsBoost, false))
            result.add(new FlightAction.Glide(a.yaw(), a.pitch(), a.firework()));
        return List.copyOf(result);
    }
    private static Trajectory expose(FlightTrajectory t) {
        var results = new ArrayList<PhysicsResult>();
        for (int i = 0; i < t.size(); i++) results.add(t.result(i));
        return new Trajectory(t.initial(), results);
    }
    public static Trajectory predict(FlightContext c, FlightAction.Glide a) {
        return expose(new VanillaGlideMode(c.movement()).predict(c.world(), c.state(), a.yaw(), a.pitch(),
                c.attached() || a.firework(), 24, c.target(), c.emergency()));
    }
    public static List<Trajectory> recover(FlightContext c, FlightAction.Glide a, Trajectory main) {
        var safety = new FlightSafety(c.movement(), c.damage(), c.health());
        var results = new ArrayList<Trajectory>();
        for (float pitch : VanillaGlideMode.PITCHES) {
            var checked = safety.simulate(c.world(), main.finalState(), safety.stepper(a.yaw(), pitch,
                    c.attached() || a.firework()), 60, a.yaw(), pitch, c.attached() || a.firework(), true);
            Trajectory t = expose(checked.trajectory());
            results.add(new Trajectory(t.initial(), t.results(), new FlightAction.Glide(a.yaw(), pitch, a.firework())));
        }
        return List.copyOf(results);
    }
    public static boolean known(FlightContext c, PhysicsState before, PhysicsResult after) {
        return after.position().isFinite() && after.velocity().isFinite()
                && FlightSafety.knownStep(c.world(), before, after);
    }
    public static DamageRule.Damage wall(FlightContext c, FlightAction action, PhysicsState before, PhysicsResult after) {
        PhysicsResult clipped = after;
        if (action instanceof FlightAction.Glide a && (c.attached() || a.firework()))
            clipped = new GlidingPhysics(a.yaw(), a.pitch(), false, c.movement().gravity(),
                    c.movement().slowFalling()).step(c.world(), before, .299, .6);
        double speed = Math.hypot(before.velocity().x, before.velocity().z);
        var d = DamageModel.wall(after.collidedX() || after.collidedZ(), speed < .003 ? 0 : speed,
                Math.hypot(clipped.velocity().x, clipped.velocity().z), c.damage());
        return new DamageRule.Damage(d.raw(), d.reduced());
    }
    public static DamageRule.Damage fall(FlightContext c, FlightAction action, PhysicsState before, PhysicsResult after) {
        double distance = before.velocity().y > -.5 ? Math.min(1, before.fallDistance()) : before.fallDistance();
        distance += (float) Math.max(0, before.position().y - after.position().y);
        String block = c.world().getBlockStateAt(after.position().add(0, -(double) .2f, 0))
                .blockName().replace("minecraft:", "");
        double multiplier = switch (block) {
            case "slime_block", "powder_snow" -> 0;
            case "hay_block", "honey_block" -> (double) .2f;
            default -> 1;
        };
        if (block.endsWith("_bed")) distance *= .5;
        var d = DamageModel.fall(after.onGround() && !c.movement().slowFalling() ? distance : 0,
                multiplier, c.movement(), c.damage());
        return new DamageRule.Damage(d.raw(), d.reduced());
    }
    public static double controls(FlightContext c, FlightAction.Glide a) {
        return BuiltinFlightCost.controls(a.yaw(), a.pitch(), c.yaw(), c.pitch(), a.firework());
    }
    public static double approach(FlightContext c, FlightAction.Glide a, Trajectory t) {
        if (t.results().isEmpty()) return Double.POSITIVE_INFINITY;
        var iterator = t.results().iterator();
        FlightTrajectory internal = FlightTrajectory.simulate(c.world(), t.initial(),
                (world, state, width, height) -> iterator.next(), t.results().size(), (state, result) -> false);
        double distance = VanillaGlideMode.horizontalDistance(c.state().position(), c.target());
        double speed = Math.hypot(c.state().velocity().x, c.state().velocity().z);
        double altitude = FlightSelector.routeAltitude(c.world(), c.state().position(), c.target());
        boolean descending = distance <= Math.max(48, (c.state().position().y - c.target().y) * 5 + speed * 24);
        boolean landing = descending && distance <= Math.max(c.radius() + 16, speed * 24)
                && c.state().position().y <= c.target().y + 12;
        double desiredAltitude = descending
                ? Math.min(altitude, c.target().y + Math.max(0, distance - c.radius()) / 5) : altitude;
        // Emergency ranking must retain damaged candidates rather than applying cruise's hard collision veto.
        if (c.emergency()) return t.finalState().position().distance(c.target())
                + BuiltinFlightCost.emergencyControls(a.yaw(), a.pitch(), c.yaw(), c.pitch());
        return BuiltinFlightCost.evaluate(c.world(), internal, c.target(), c.radius(),
                desiredAltitude, a.yaw(), a.pitch(),
                c.attached() || a.firework(), landing, false, t.results().size(), false);
    }
}
