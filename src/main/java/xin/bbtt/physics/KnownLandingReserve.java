package xin.bbtt.physics;

import java.util.Collection;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashSet;
import org.joml.Vector3d;
import xin.bbtt.world.World;

/** Bounded known-ground resource gate, not a terminal/stalled-state safety proof.
 * Selects the nearest currently viable supported observation, with bounded loaded
 * route/landing geometry and a distance-scalable resource work estimate.
 * Call on resource/route changes or before irreversible exposure, not per control candidate.
 */
public final class KnownLandingReserve {
    // 66 memory observations + underfoot + complete 9x9 loaded probe grid.
    public static final int MAX_SITES = 148;
    private static final double MAX_DISTANCE = 1024;

    private KnownLandingReserve() {}

    /** A low roof alone does not opt ordinary supported flight into lava policy.
     * Only loaded columns with observed lava before solid support establish exposure.
     */
    public static boolean nearLava(World world, PhysicsState state) {
        boolean[] found = {false};
        world.readTerrain(() -> {
            var p = state.position();
            for (int dx = -32; dx <= 32; dx += 8) for (int dz = -32; dz <= 32; dz += 8) {
                int x = (int) Math.floor(p.x) + dx, z = (int) Math.floor(p.z) + dz;
                if (!world.chunkLoaded(x >> 4, z >> 4)) continue;
                for (int y = (int) Math.floor(p.y); y >= World.getMinWorldY(); y--) {
                    var cell = new Vector3d(x + .5, y, z + .5);
                    String name = world.getBlockStateAt(cell).blockName();
                    if ("lava".equals(name) || "minecraft:lava".equals(name)) {
                        found[0] = true;
                        return true;
                    }
                    if (world.isSolid(cell)) break;
                }
            }
            return false;
        });
        return found[0];
    }

    /** Resource admission is not a full terminal-policy search. DeploymentSafety still
     * independently proves both metadata branches before this result can be installed.
     */
    public static boolean launchReserve(World world, PhysicsState state, MovementParameters parameters,
            HazardRecovery.Context resources, float yaw, float pitch, java.util.List<Vector3d> sites) {
        return launchAssessment(world, state, parameters, resources, yaw, pitch, sites).canContinue();
    }

    public enum LaunchReason { ADMITTED, BLOCKED_EXCURSION, NO_ANCHOR, UNPROVEN_RETURN,
        INVENTORY_MARGIN, INVALID_RESOURCES }

    /** Observations, not a diagnosis of the unavailable actual180 field state.
     * Attempts retain the actual pitches tried; reserve is the authoritative first
     * terrain-safe excursion result. A failed proof must not be called a fuel shortage.
     */
    public record LaunchResult(LaunchReason reason, java.util.List<Float> attemptedPitches,
                               int excursionTicks, int consumedRockets, int cooldownTicks,
                               long boostTicks, Result reserve) {
        public LaunchResult { attemptedPitches = java.util.List.copyOf(attemptedPitches); }
        public boolean canContinue() { return reason == LaunchReason.ADMITTED; }
    }

    public static LaunchReason launchReason(Result result) {
        return switch (result.status()) {
            case NO_SITE -> LaunchReason.NO_ANCHOR;
            case INVALID_RESOURCES -> LaunchReason.INVALID_RESOURCES;
            case BLOCKED_OR_UNKNOWN, UNPROVEN -> LaunchReason.UNPROVEN_RETURN;
            case VALID -> result.canContinue() ? LaunchReason.ADMITTED : LaunchReason.INVENTORY_MARGIN;
        };
    }

    public static LaunchResult launchAssessment(World world, PhysicsState state, MovementParameters parameters,
            HazardRecovery.Context resources, float yaw, float pitch, java.util.List<Vector3d> sites) {
        if (state == null) return new LaunchResult(LaunchReason.BLOCKED_EXCURSION,
                java.util.List.of(), 0, 0, resources.cooldownTicks(), resources.guaranteedBoostTicks(), null);
        // Validate before scheduling: invalid confirmed boost must not throw or become free fuel.
        var validation = assess(world, state, parameters, resources, java.util.List.of(), 10, 1);
        if (!validation.resourcesValid()) return new LaunchResult(LaunchReason.INVALID_RESOURCES,
                java.util.List.of(), 0, 0, resources.cooldownTicks(), resources.guaranteedBoostTicks(), validation);
        var attempts = new java.util.ArrayList<Float>();
        LaunchResult last = null;
        // Only geometric blockage permits a climbing alternative; resource rejection remains authoritative.
        for (float excursionPitch : new float[] {pitch, -10, -30}) {
            attempts.add(excursionPitch);
            last = launchExcursion(world, state, parameters, resources, yaw, excursionPitch, sites, attempts);
            if (last.reason() != LaunchReason.BLOCKED_EXCURSION) return last;
        }
        return last;
    }

    private static LaunchResult launchExcursion(World world, PhysicsState state, MovementParameters parameters,
            HazardRecovery.Context resources, float yaw, float pitch, java.util.List<Vector3d> sites,
            java.util.List<Float> attempts) {
        int[] flights = resources.rocketFlightDurations();
        var schedule = FireworkSchedule.confirmed(flights, flights.length, resources.cooldownTicks(),
                resources.guaranteedBoostTicks(), 0);
        // Account for deployment and the first outward reuse window, then retain a return
        // estimate plus an unused safety rocket. A supported origin is not free launch.
        for (int tick = 0; tick < 40; tick++) {
            var resource = schedule.nextTick();
            var next = new GlidingPhysics(yaw, pitch, resource.powered(), PlayerInput.NONE, parameters)
                    .step(world, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            if (!FlightSafety.knownStep(world, state, next) || next.onGround()
                    || next.collidedX() || next.collidedZ()
                    || FlightHazards.damage(world, state, next, DamageParameters.NONE, true).raw() > 0
                    || FlightHazards.damage(world, state, next, DamageParameters.NONE, false).raw() > 0) {
                var checkpoint = schedule.checkpoint();
                return new LaunchResult(LaunchReason.BLOCKED_EXCURSION, attempts, tick + 1,
                        checkpoint.consumed(), checkpoint.cooldownTicks(), checkpoint.boostTicks(), null);
            }
            state = FlightSafety.nextState(next);
        }
        var checkpoint = schedule.checkpoint();
        int[] remaining = java.util.Arrays.copyOfRange(flights, checkpoint.consumed(), flights.length);
        var reserve = new HazardRecovery.Context(remaining, checkpoint.cooldownTicks(),
                checkpoint.boostTicks() > 0, resources.memory(), checkpoint.boostTicks());
        var result = assess(world, state, parameters, reserve, sites, 10, 1);
        return new LaunchResult(launchReason(result), attempts, 40, checkpoint.consumed(),
                checkpoint.cooldownTicks(), checkpoint.boostTicks(), result);
    }

    public enum Status { VALID, NO_SITE, INVALID_RESOURCES, BLOCKED_OR_UNKNOWN, UNPROVEN }

    /** requiredRockets is an estimated sufficient inventory prefix, not a mathematical
     * minimum. -1 means estimation failure. Loaded corridors check route geometry;
     * discounted scheduled thrust estimates work, but does not prove touchdown. Surplus is inventory minus required minus margin.
     * Geometry validity is a loaded, collision/hazard-free vertical landing corridor only;
     * it does not assert dynamic turn feasibility or a damage-free touchdown.
     */
    public record Result(Status status, Vector3d returnSite, int requiredRockets,
                         int safetyMarginRockets, int surplusRockets,
                         boolean resourcesValid, boolean geometryValid) {
        public Result { returnSite = returnSite == null ? null : new Vector3d(returnSite); }
        @Override public Vector3d returnSite() {
            return returnSite == null ? null : new Vector3d(returnSite);
        }
        /** False means the gate must not authorize new irreversible exposure. */
        public boolean canContinue() { return status == Status.VALID && surplusRockets > 0; }
        public boolean shouldReturn() { return !canContinue(); }
    }

    /** Inventory order must match actual next-use order. Confirmed boost comes from
     * Context, never an attempted use. Unknown attached age receives at most Context's
     * conservative next-tick credit. Delay is a caller-provided acknowledgement bound.
     * No site discovery, terminal rollout, or stalled-branch search is performed here.
     */
    public static Result assess(World world, PhysicsState state, MovementParameters movement,
            HazardRecovery.Context resources, Collection<Vector3d> knownSites,
            int confirmationDelayTicks, int safetyMarginRockets) {
        if (safetyMarginRockets < 0) throw new IllegalArgumentException("Negative safety margin");
        int[] flights = resources.rocketFlightDurations();
        boolean valid = confirmationDelayTicks >= 0 && confirmationDelayTicks <= 600
                && resources.guaranteedBoostTicks() >= 0
                && state.position().isFinite() && state.velocity().isFinite()
                && Double.isFinite(state.fallDistance());
        for (int flight : flights) valid &= flight >= 0 && flight <= 255;
        if (!valid) return failure(Status.INVALID_RESOURCES, null, safetyMarginRockets, false, false);
        Result[] answer = {failure(Status.NO_SITE, null, safetyMarginRockets, true, false)};
        world.readTerrain(() -> {
            var unique = new LinkedHashSet<Vector3d>();
            // Raw duplicates never consume the geometry/trajectory budget.
            for (Vector3d remembered : knownSites) {
                if (remembered == null || !remembered.isFinite()) continue;
                if (unique.size() < MAX_SITES) unique.add(new Vector3d(remembered));
            }
            var candidates = new ArrayList<Vector3d>();
            for (Vector3d remembered : unique) {
                Vector3d site = VanillaGlideMode.landingSite(world, remembered.x, remembered.z, remembered.y);
                if (site != null && Math.abs(site.y - remembered.y) <= 1.0e-6) candidates.add(site);
            }
            candidates.sort(Comparator.comparingDouble(site ->
                    Math.hypot(site.x - state.position().x, site.z - state.position().z)));
            Result firstFailure = null;
            for (Vector3d site : candidates) {
                Result candidate = failure(Status.BLOCKED_OR_UNKNOWN, site, safetyMarginRockets, true, false);
                if (state.position().distance(site) > MAX_DISTANCE) {
                    candidate = failure(Status.UNPROVEN, site, safetyMarginRockets, true, false);
                } else {
                    // Loaded route geometry is independent of resource work. A target-plane
                    // rollout with fixed pitch can miss a perfectly usable nearby anchor,
                    // especially while reversing. Exact touchdown is an execution concern.
                    for (double extra : new double[] {0, 8, 16}) {
                        double height = Math.max(state.position().y, site.y + 4) + extra;
                        Vector3d departure = new Vector3d(state.position().x, height, state.position().z);
                        Vector3d approach = new Vector3d(site.x, height, site.z);
                        if (!corridor(world, state.position(), departure)
                                || !corridor(world, departure, approach)
                                || !corridor(world, approach, site)) continue;
                        double dx = site.x - state.position().x, dz = site.z - state.position().z;
                        double distance = Math.hypot(dx, dz);
                        double away = distance == 0 ? 0 : Math.max(0,
                                -(state.velocity().x * dx + state.velocity().z * dz) / distance);
                        int count = FireworkSchedule.reserveRockets(flights, resources.cooldownTicks(),
                                resources.guaranteedBoostTicks(), confirmationDelayTicks, distance,
                                height - state.position().y, away);
                        candidate = failure(Status.UNPROVEN, site, safetyMarginRockets, true, true);
                        if (count >= 0) {
                            answer[0] = new Result(Status.VALID, site, count, safetyMarginRockets,
                                    flights.length - count - safetyMarginRockets, true, true);
                            return true;
                        }
                    }
                }
                if (firstFailure == null) firstFailure = candidate;
            }
            if (firstFailure != null) answer[0] = firstFailure;
            return false;
        });
        return answer[0];
    }

    private static Result failure(Status status, Vector3d site, int margin, boolean resources, boolean geometry) {
        return new Result(status, site, -1, margin, Integer.MIN_VALUE, resources, geometry);
    }

    static boolean corridor(World world, Vector3d from, Vector3d site) {
        double distance = from.distance(site);
        if (!Double.isFinite(distance) || distance > MAX_DISTANCE) return false;
        int segments = Math.max(1, (int) Math.ceil(distance / 4));
        Vector3d previous = from;
        for (int i = 1; i <= segments; i++) {
            if (Thread.currentThread().isInterrupted()) return false;
            Vector3d next = new Vector3d(from).lerp(site, (double) i / segments);
            if (!LocalAirRoute.visible(world, previous, next, PlayerPose.GLIDING)) return false;
            PhysicsState before = new PhysicsState(previous, new Vector3d(), false, 0, PlayerPose.GLIDING);
            PhysicsResult after = new PhysicsResult(next, new Vector3d(), false, false, false, 0, PlayerPose.GLIDING);
            if (!FlightSafety.knownStep(world, before, after)
                    || FlightHazards.damage(world, before, after, DamageParameters.NONE, true).raw() > 0
                    || FlightHazards.damage(world, before, after, DamageParameters.NONE, false).raw() > 0) return false;
            previous = next;
        }
        return true;
    }
}
