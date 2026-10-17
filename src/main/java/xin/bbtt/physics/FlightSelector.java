package xin.bbtt.physics;

import java.util.Objects;
import static xin.bbtt.physics.VanillaGlideMode.*;
import xin.bbtt.physics.ElytraFlightController.Decision;
import xin.bbtt.physics.ElytraFlightController.Phase;
import xin.bbtt.physics.FlightSafety.Impact;
import xin.bbtt.physics.FlightSafety.EmergencyCost;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Bounded trajectory steering. The caller owns rocket lifetime and flight lifecycle. */
final class FlightSelector {
    private static final int HORIZON = 24;
    // Receding-horizon corridor steering: six travel ticks covers several decision
    // updates without forcing a single constant heading through a future bend.
    private static final int LOW_HORIZON = 6;
    private final VanillaGlideMode mode;
    private final java.util.List<FlightMode> modes;
    private final FlightSafety safety;
    private final MovementParameters movement;
    private final double health;

    FlightSelector() {
        this(0.08, false);
    }

    FlightSelector(double gravity, boolean slowFalling) {
        this(new MovementParameters(MovementParameters.DEFAULT.movementSpeed(),
                MovementParameters.DEFAULT.sprintingMovementSpeed(), MovementParameters.DEFAULT.jumpStrength(),
                0, gravity, .6, .3, slowFalling, -1, true), DamageParameters.NONE, 20);
    }

    /** Damage-aware emergency prediction; unknown absorption is conservatively ignored. */
    FlightSelector(MovementParameters movement, DamageParameters mitigation, double health) {
        this.movement = Objects.requireNonNull(movement);
        this.mode = new VanillaGlideMode(movement);
        this.modes = java.util.List.of(mode);
        Objects.requireNonNull(mitigation);
        this.safety = new FlightSafety(movement, mitigation, health, PITCHES);
        this.health = health;
    }

    public Decision decide(World world, PhysicsState state, float yaw, float pitch,
            boolean attached, Vector3d target, int fireworks, double radius) {
        return decide(world, state, yaw, pitch, attached, new FixedFlightTarget(target), fireworks, radius);
    }

    public Decision decide(World world, PhysicsState state, float yaw, float pitch,
            boolean attached, FlightTarget destination, int fireworks, double radius) {
        if (destination instanceof WaypointFlightTarget route)
            return decideLow(world, state, yaw, pitch, attached, route, fireworks, radius);
        Vector3d target = destination.aim(state);
        Objects.requireNonNull(world, "world");
        Objects.requireNonNull(state, "state");
        Objects.requireNonNull(target, "target");
        if (!state.position().isFinite() || !state.velocity().isFinite() || !target.isFinite()
                || !Float.isFinite(yaw) || !Float.isFinite(pitch)
                || !Double.isFinite(radius) || radius <= 0 || fireworks < 0) {
            throw new IllegalArgumentException("Flight inputs must be finite, radius positive and fireworks nonnegative");
        }
        if (state.onGround()) return new Decision(normalize(yaw), 0, false, Phase.LANDED);

        Vector3d position = state.position();
        double distance = horizontalDistance(position, target);
        double speed = Math.hypot(state.velocity().x, state.velocity().z);
        double cruiseAltitude = routeAltitude(world, position, target);
        boolean descending = (destination instanceof FixedFlightTarget || destination.lost())
                && distance <= Math.max(48, (position.y - target.y) * 5 + speed * HORIZON);
        boolean landing = descending && distance <= Math.max(radius + 16, speed * HORIZON)
                && position.y <= target.y + 12;
        Phase phase = Phase.CRUISE;
        if (landing) phase = Phase.LANDING;
        else if (descending) phase = Phase.DESCENT;
        else if (position.y < cruiseAltitude - 3) phase = Phase.CLIMB;
        double desiredAltitude = descending
                ? Math.min(cruiseAltitude, target.y + Math.max(0, distance - radius) / 5)
                : cruiseAltitude;
        boolean needsBoost = !descending && (position.y < cruiseAltitude - 3 || speed < 0.65);
        float heading = distance < 0.01 ? normalize(yaw)
                : (float) Math.toDegrees(Math.atan2(position.x - target.x, target.z - position.z));
        Decision best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        // Policy seam: cruise deliberately uses cost-only ordering in A.
        for (FlightMode owner : modes) {
            for (FlightMode.Candidate candidate : owner.candidates(yaw, heading,
                    !attached && fireworks > 0 && needsBoost, false)) {
                float candidateYaw = candidate.yaw(), candidatePitch = candidate.pitch();
                boolean firework = candidate.firework();
                double score = evaluate(owner, world, state, target, radius, desiredAltitude,
                        candidateYaw, candidatePitch, attached || firework, landing, false);
                if (!Double.isFinite(score)) continue;
                score += BuiltinFlightCost.controls(candidateYaw, candidatePitch, yaw, pitch, firework);
                if (best != null && !FlightRanking.COST_ONLY.better(score, bestScore)) continue;
                bestScore = score;
                best = new Decision(candidateYaw, candidatePitch, firework, phase);
            }
        }
        // Preserve normal decisions. Only exhausted, unpowered flight may accept
        // a safe touchdown outside the destination instead of abandoning steering.
        if (best == null && fireworks == 0 && !attached) {
            for (FlightMode owner : modes) {
                for (FlightMode.Candidate candidate : owner.candidates(yaw, heading, false, false)) {
                    float candidateYaw = candidate.yaw(), candidatePitch = candidate.pitch();
                    double score = evaluate(owner, world, state, target, radius, desiredAltitude,
                            candidateYaw, candidatePitch, false, false, true);
                    if (score >= bestScore) continue;
                    bestScore = score;
                    best = new Decision(candidateYaw, candidatePitch, false, Phase.FORCED_LANDING);
                }
            }
        }
        if (best == null) throw new IllegalStateException("No collision-free flight trajectory");
        return best;
    }

    /** Corridor steering is isolated so fixed-target arithmetic stays untouched. */
    private Decision decideLow(World world, PhysicsState state, float yaw, float pitch,
            boolean attached, WaypointFlightTarget route, int fireworks, double radius) {
        return decideLow(world, state, yaw, pitch, attached, route, fireworks, radius, decision -> true);
    }

    Decision decideLow(World world, PhysicsState state, float yaw, float pitch,
            boolean attached, WaypointFlightTarget route, int fireworks, double radius,
            java.util.function.Predicate<Decision> allowed) {
        Objects.requireNonNull(world, "world");
        Objects.requireNonNull(state, "state");
        if (!state.position().isFinite() || !state.velocity().isFinite()
                || !Float.isFinite(yaw) || !Float.isFinite(pitch)
                || !Double.isFinite(radius) || radius <= 0 || fireworks < 0)
            throw new IllegalArgumentException("Flight inputs must be finite, radius positive and fireworks nonnegative");
        if (state.onGround()) return new Decision(normalize(yaw), 0, false, Phase.LANDED);
        route.advance(world, state, movement);
        Vector3d target = route.aim(state);
        float heading = LowAltitudePlanner.yaw(new Vector3d(target).sub(state.position()));
        Decision best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        for (FlightMode.Candidate candidate : mode.candidates(yaw, heading,
                !attached && fireworks > 0, false)) {
            boolean boosted = attached || candidate.firework();
            var checked = safety.simulate(world, state,
                    mode.stepper(candidate.yaw(), candidate.pitch(), boosted), LOW_HORIZON,
                    candidate.yaw(), candidate.pitch(), boosted, false);
            if (checked.terminalImpact() != null && (checked.terminalImpact().unknown()
                    || checked.terminalImpact().raw() > 0)) continue;
            var trajectory = checked.trajectory();
            if (!route.accepts(trajectory)) continue;
            boolean valid = true;
            double score = 0;
            for (int tick = 0; tick < trajectory.size(); tick++) {
                var result = trajectory.result(tick);
                if (result.collidedX() || result.collidedZ()
                        || result.onGround() && (!route.landed()
                        || result.position().distance(route.goal()) > radius)) {
                    valid = false;
                    break;
                }
                // No terrain-top reserve or fixed clearance penalty in a corridor.
                if (route.pursuit()) {
                    Vector3d liveTarget = new Vector3d(target);
                    if (route.pursuitEndpoint())
                        liveTarget.fma(tick + 1, route.velocity());
                    // Integrated tracking error makes powered and unpowered
                    // trajectories compete on actual progress, not a speed gate.
                    score += result.position().distance(liveTarget);
                } else score += Math.abs(result.position().y - target.y) / HORIZON;
            }
            if (!valid || trajectory.size() == 0) continue;
            PhysicsState end = checked.finalState();
            if (route.pursuit()) {
                // Track the moving follow offset, not a stationary intercept. The
                // velocity error brakes before passing it and avoids endpoint hunting.
                // Interior corridor vertices are fixed geometry, not moving
                // entities: extrapolating a ridge corner can put the scored goal
                // back inside the ridge. Match entity motion only at the endpoint.
                Vector3d velocity = route.pursuitEndpoint() ? route.velocity() : new Vector3d();
                Vector3d movingTarget = new Vector3d(target).fma(LOW_HORIZON, velocity);
                Vector3d desiredVelocity = new Vector3d(movingTarget).sub(end.position()).div(LOW_HORIZON).add(velocity);
                Vector3d actualVelocity = end.velocity();
                score += end.position().distance(movingTarget) * 3
                        + actualVelocity.distance(desiredVelocity) * LOW_HORIZON;
            } else score += end.position().distance(target) * 3;
            score += BuiltinFlightCost.controls(candidate.yaw(), candidate.pitch(), yaw, pitch, candidate.firework());
            if (score < bestScore) {
                Decision decision = new Decision(candidate.yaw(), candidate.pitch(), candidate.firework(),
                        route.landed() ? Phase.LANDING : Phase.CRUISE);
                if (!allowed.test(decision)) continue;
                bestScore = score;
                best = decision;
            }
        }
        if (best == null) throw new IllegalStateException("No safe low-altitude trajectory");
        return best;
    }

    /** Explicit emergency entry point; normal cruise decisions are unchanged. */
    public Decision forceLand(World world, PhysicsState state, float yaw, float pitch,
            boolean attached, int fireworks, double radius) {
        return forceLand(world, state, yaw, pitch, attached, fireworks, radius, null);
    }

    public Decision forceLand(World world, PhysicsState state, float yaw, float pitch,
            boolean attached, int fireworks, double radius, xin.bbtt.movements.LandingMemory memory) {
        if (fireworks < 0) throw new IllegalArgumentException("Fireworks must be nonnegative");
        int[] flights = new int[fireworks];
        java.util.Arrays.fill(flights, 1);
        return forceLand(world, state, yaw, pitch, attached, flights, radius, memory, mode.landingSearch);
    }

    /** Incremental search and actual inventory durations, owned by the flying movement. */
    public Decision forceLand(World world, PhysicsState state, float yaw, float pitch,
            boolean attached, int[] flights, double radius, xin.bbtt.movements.LandingMemory memory,
            LandingSearch search) {
        Vector3d target = search.select(world, state, flights, attached, movement, LandingSearch.BUDGET);
        boolean remembered = target == null;
        if (remembered) target = memory == null ? null : memory.nearest(state.position());
        search.selectedMemory(remembered ? target : null);
        if (target == null) throw new IllegalStateException("No known solid emergency landing ground in loaded terrain");
        if (!Float.isFinite(yaw) || !Float.isFinite(pitch) || !Double.isFinite(radius) || radius <= 0) {
            throw new IllegalArgumentException("Emergency controls must be finite and radius positive");
        }
        if (state.onGround()) {
            if (horizontalDistance(state.position(), target) > 1
                    || Math.abs(state.position().y - target.y) > 0.5) {
                throw new IllegalStateException("Emergency touchdown has no verified solid support");
            }
            return new Decision(normalize(yaw), 0, false, Phase.LANDED);
        }
        double distance = horizontalDistance(state.position(), target);
        boolean needsBoost = distance > mode.range(state).atHeight(target.y, 0, false);
        float heading = distance < 0.01 ? yaw
                : (float) Math.toDegrees(Math.atan2(state.position().x - target.x, target.z - state.position().z));
        return emergencyDecision(world, state, yaw, pitch, attached, target, radius, heading,
                needsBoost, needsBoost && !attached && flights.length > 0, false,
                remembered);
    }

    Decision emergencyDecision(World world, PhysicsState state, float yaw, float pitch,
            boolean attached, Vector3d target, double radius, float heading,
            boolean needsBoost, boolean firework, boolean hold) {
        return emergencyDecision(world, state, yaw, pitch, attached, target, radius, heading,
                needsBoost, firework, hold, false);
    }

    private Decision emergencyDecision(World world, PhysicsState state, float yaw, float pitch,
            boolean attached, Vector3d target, double radius, float heading,
            boolean needsBoost, boolean firework, boolean hold, boolean rememberedTarget) {
        boolean longestGlide = rememberedTarget && !attached && !firework;
        Decision best = null;
        EmergencyCost bestCost = null;
        GlideRange glide = longestGlide ? mode.range(state) : null;
        for (FlightMode owner : modes) {
            for (FlightMode.Candidate candidate : owner.candidates(yaw, heading, false, firework)) {
                float candidateYaw = candidate.yaw(), candidatePitch = candidate.pitch();
                Impact impact = safety.predictEmergencyImpact(world, state, candidateYaw, candidatePitch, attached || firework,
                        owner.stepper(candidateYaw, candidatePitch, attached || firework), owner.recovery(safety));
                double score = evaluate(owner, world, state, target, Math.min(radius, 1),
                        target.y + (needsBoost ? horizontalDistance(state.position(), target) / 5 + 8 : 0),
                        candidateYaw, candidatePitch, attached || firework, !needsBoost, false, HORIZON, true);
                int approach = 0;
                if (hold || !Double.isFinite(score)) {
                    approach = 1;
                    score = evaluate(owner, world, state, target, radius, target.y,
                            candidateYaw, candidatePitch, attached, false, true);
                }
                if (!Double.isFinite(score)) {
                    approach = 2;
                    score = BuiltinFlightCost.emergencyFallback(candidateYaw, candidatePitch,
                            yaw, heading, rememberedTarget);
                } else if (approach == 0) {
                    score += BuiltinFlightCost.emergencyControls(candidateYaw, candidatePitch, yaw, pitch);
                }
                if (longestGlide) {
                    // Damage classes/recovery stay first. Unknown approach terrain
                    // cannot prevent best-effort steering toward remembered land.
                    approach = 0;
                    score = BuiltinFlightCost.rememberedRange(glide, target.y, candidateYaw, candidatePitch, heading);
                }
                EmergencyCost cost = new EmergencyCost(impact.rank(health),
                        impact.reduced(), approach, score);
                if (bestCost != null && !FlightRanking.DAMAGE_FIRST.better(cost, bestCost)) continue;
                bestCost = cost;
                best = new Decision(candidateYaw, candidatePitch, firework, Phase.FORCED_LANDING);
            }
        }
        return best;
    }

    double emergencyDamage(World world, PhysicsState state, float yaw, float pitch, boolean attached) {
        return safety.emergencyDamage(world, state, yaw, pitch, attached);
    }

    /** Damage-first unpowered steering, also when no landing site is known. */
    public Decision emergencyHold(World world, PhysicsState state, float yaw, float pitch, boolean attached) {
        return emergencyDecision(world, state, yaw, pitch, attached, state.position(), 1, yaw, false, false, true);
    }

    private double evaluate(FlightMode owner, World world, PhysicsState initial, Vector3d target,
            double radius, double cruiseAltitude, float yaw, float pitch,
            boolean boosted, boolean landing, boolean forcedLanding) {
        return evaluate(owner, world, initial, target, radius, cruiseAltitude, yaw, pitch,
                boosted, landing, forcedLanding, HORIZON, false);
    }

    private double evaluate(FlightMode owner, World world, PhysicsState initial, Vector3d target,
            double radius, double cruiseAltitude, float yaw, float pitch,
            boolean boosted, boolean landing, boolean forcedLanding, int horizon, boolean emergencyApproach) {
        FlightTrajectory trajectory = owner.predict(world, initial, yaw, pitch, boosted, horizon,
                target, emergencyApproach);
        return BuiltinFlightCost.evaluate(world, trajectory, target, radius, cruiseAltitude,
                yaw, pitch, boosted, landing, forcedLanding, horizon, emergencyApproach);
    }

    /** Scan bounded route tiles, never treating unloaded terrain as known air. */
    static double routeAltitude(World world, Vector3d position, Vector3d target) {
        // Below a ceiling, the roof is an obstacle, not terrain to cruise over.
        // Keep the non-ceiling path (including arithmetic order) unchanged.
        boolean belowRoof = xin.bbtt.listeners.RegistryDataListener.hasCeiling()
                && !CeilingCollision.isAboveRoof(world, position);
        double scanTop = belowRoof ? Math.max(position.y, target.y) : World.getMaxWorldY();
        double scanBottom = !belowRoof && xin.bbtt.listeners.RegistryDataListener.hasCeiling()
                ? CeilingCollision.roofTop(world, position.x, position.z).orElse(World.getMinWorldY())
                : World.getMinWorldY();
        double distance = horizontalDistance(position, target);
        int tiles = Math.min(64, Math.max(1, (int) Math.ceil(distance / 16)));
        double scannedDistance = Math.min(distance, 1024);
        double highest = !belowRoof && xin.bbtt.listeners.RegistryDataListener.hasCeiling()
                ? Math.max(target.y, scanBottom) : target.y;
        boolean unknown = distance > scannedDistance;
        for (int tile = 0; tile < tiles; tile++) {
            double start = distance == 0 ? 0 : scannedDistance * tile / tiles / distance;
            double end = distance == 0 ? 0 : scannedDistance * (tile + 1) / tiles / distance;
            double firstX = position.x + (target.x - position.x) * start;
            double firstZ = position.z + (target.z - position.z) * start;
            double lastX = position.x + (target.x - position.x) * end;
            double lastZ = position.z + (target.z - position.z) * end;
            double minX = Math.min(firstX, lastX) - HALF_WIDTH;
            double maxX = Math.max(firstX, lastX) + HALF_WIDTH;
            double minZ = Math.min(firstZ, lastZ) - HALF_WIDTH;
            double maxZ = Math.max(firstZ, lastZ) + HALF_WIDTH;
            for (int chunkX = (int) Math.floor(minX) >> 4; chunkX <= (int) Math.floor(maxX) >> 4; chunkX++) {
                for (int chunkZ = (int) Math.floor(minZ) >> 4; chunkZ <= (int) Math.floor(maxZ) >> 4; chunkZ++) {
                    if (!world.chunkLoaded(chunkX, chunkZ)) {
                        unknown = true;
                        continue;
                    }
                    highest = Math.max(highest, world.findHighestCollisionTop(
                            Math.max(minX, chunkX * 16.0), scanBottom,
                            Math.max(minZ, chunkZ * 16.0), Math.min(maxX, chunkX * 16.0 + 16),
                            scanTop, Math.min(maxZ, chunkZ * 16.0 + 16)).orElse(highest));
                }
            }
        }
        // Distant unknown terrain raises the overworld reserve, not a global stop.
        // evaluate still rejects any trajectory entering an unloaded chunk at any altitude.
        double altitude = highest + 16;
        if (belowRoof) return Math.min(scanTop, altitude);
        if (unknown) altitude = Math.max(altitude, Math.max(128, target.y + 32));
        return Math.min(World.getMaxWorldY() - HEIGHT - 2, altitude);
    }

}
