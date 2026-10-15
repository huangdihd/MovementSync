package xin.bbtt.physics;

import java.util.OptionalDouble;
import java.util.Set;
import org.joml.Vector3d;
import org.joml.Vector3i;
import org.junit.jupiter.api.Test;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class ElytraFlightControllerTest {
    private final ElytraFlightController controller = new ElytraFlightController();

    private static class Terrain extends World {
        int queries;
        boolean wall;

        @Override
        public boolean chunkLoaded(int chunkX, int chunkZ) { return true; }

        @Override
        public boolean isBoxColliding(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ) {
            queries++;
            return minY < 0 || (wall && minX < 2 && maxX > -2 && maxZ > 8 && minZ < 9 && minY < 30);
        }

        @Override
        public OptionalDouble findHighestCollisionTopExcluding(double minX, double minY, double minZ,
                double maxX, double maxY, double maxZ, Set<Vector3i> excluded) {
            return minY <= 0 && maxY >= 0 ? OptionalDouble.of(0) : OptionalDouble.empty();
        }
    }

    private static PhysicsState airborne() {
        return new PhysicsState(new Vector3d(0, 20, 0), new Vector3d(0, 0, 0.8), false);
    }

    @org.junit.jupiter.params.ParameterizedTest
    @org.junit.jupiter.params.provider.ValueSource(booleans = {false, true})
    void exhaustedFlightControlsTouchdownOutsideTarget(boolean uneven) {
        Terrain world = new Terrain() {
            private double top(double minZ, double maxZ) {
                return uneven && maxZ > 4 && minZ < 8 ? 2 : 0;
            }
            @Override public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) {
                return minY < top(minZ, maxZ);
            }
            @Override public OptionalDouble findHighestCollisionTopExcluding(
                    double minX, double minY, double minZ, double maxX, double maxY, double maxZ,
                    Set<Vector3i> excluded) {
                double top = top(minZ, maxZ);
                if (minY <= top && maxY >= top) return OptionalDouble.of(top);
                return super.findHighestCollisionTopExcluding(minX, minY, minZ, maxX, maxY, maxZ, excluded);
            }
        };
        PhysicsState state = new PhysicsState(new Vector3d(0, 1, 0), new Vector3d(0, -0.2, 0.3), false);
        Vector3d target = new Vector3d(0, 0, 800);
        boolean forced = false;
        float yaw = 0;
        float pitch = 0;
        for (int tick = 0; tick < 200 && !state.onGround(); tick++) {
            var decision = controller.decide(world, state, yaw, pitch, false, target, 0, 16);
            forced |= decision.phase() == ElytraFlightController.Phase.FORCED_LANDING;
            yaw = decision.yaw();
            pitch = decision.pitch();
            PhysicsResult result = new GlidingPhysics(yaw, pitch, false).step(world, state, 0.299, 0.6);
            assertFalse(result.collidedX() || result.collidedZ());
            if (result.onGround()) {
                assertTrue(Math.abs(state.velocity().y) <= 0.5);
                assertTrue(Math.hypot(state.velocity().x, state.velocity().z) <= 0.8);
            }
            state = new PhysicsState(result.position(), result.velocity(), result.onGround());
        }
        assertTrue(forced);
        assertTrue(state.onGround());
        assertTrue(state.position().distance(target) > 16);
    }

    @Test
    void availableRocketPreventsForcedLanding() {
        var decision = controller.decide(new Terrain(),
                new PhysicsState(new Vector3d(0, 1, 0), new Vector3d(0, -0.2, 0.3), false),
                0, 0, false, new Vector3d(0, 0, 800), 1, 16);
        assertNotEquals(ElytraFlightController.Phase.FORCED_LANDING, decision.phase());
        assertTrue(decision.firework());
    }

    @Test
    void deterministicAndDoesNotMutateInputs() {
        Terrain world = new Terrain();
        PhysicsState state = airborne();
        Vector3d target = new Vector3d(0, 0, 120);
        var first = controller.decide(world, state, 0, 0, false, target, 5, 3);
        assertEquals(first, controller.decide(world, state, 0, 0, false, target, 5, 3));
        assertEquals(airborne(), state);
        assertEquals(new Vector3d(0, 0, 120), target);
        assertTrue(Float.isFinite(first.yaw()));
        assertTrue(Float.isFinite(first.pitch()));
        assertTrue(world.queries < 60_000, "Candidate search must stay bounded");
    }

    @Test
    void noDuplicateOrUnavailableRocketRequests() {
        Vector3d target = new Vector3d(0, 0, 120);
        assertFalse(controller.decide(new Terrain(), airborne(), 0, 0, true, target, 5, 3).firework());
        assertFalse(controller.decide(new Terrain(), airborne(), 0, 0, false, target, 0, 3).firework());
    }

    @Test
    void obstacleChangesThePredictedTrajectory() {
        Terrain clear = new Terrain();
        Terrain blocked = new Terrain();
        blocked.wall = true;
        Vector3d target = new Vector3d(0, 0, 120);
        var straight = controller.decide(clear, airborne(), 0, 0, false, target, 0, 3);
        var avoidance = controller.decide(blocked, airborne(), 0, 0, false, target, 0, 3);
        assertNotEquals(straight, avoidance);
        PhysicsState state = airborne();
        for (int tick = 0; tick < 24; tick++) {
            PhysicsResult result = new GlidingPhysics(avoidance.yaw(), avoidance.pitch(), false)
                    .step(blocked, state, 0.299, 0.6);
            assertFalse(result.collidedX() || result.collidedZ());
            state = new PhysicsState(result.position(), result.velocity(), result.onGround());
        }
    }

    @Test
    void nearbyTargetEntersLandingWithoutSpendingRocket() {
        var decision = controller.decide(new Terrain(),
                new PhysicsState(new Vector3d(0, 3, 0), new Vector3d(0, -0.1, 0.3), false),
                0, 0, false, new Vector3d(0, 0, 8), 10, 3);
        assertEquals(ElytraFlightController.Phase.LANDING, decision.phase());
        assertFalse(decision.firework());
    }

    @Test
    void steeringKeepsYawContinuousAcrossFullTurns() {
        float yaw = 1080;
        var decision = controller.decide(new Terrain(), airborne(), yaw, 0, false,
                new Vector3d(0, 0, 400), 0, 16);
        assertTrue(Math.abs(decision.yaw() - yaw) <= 180);
        assertTrue(decision.yaw() > 720, "Do not modulo the packet yaw");
    }

    @Test
    void groundedStateNeverLaunches() {
        var decision = controller.decide(new Terrain(),
                new PhysicsState(new Vector3d(), new Vector3d(), true),
                720, 0, false, new Vector3d(0, 0, 100), 10, 3);
        assertEquals(ElytraFlightController.Phase.LANDED, decision.phase());
        assertFalse(decision.firework());
        assertEquals(0, decision.yaw());
    }

    @Test
    void completesFourHundredBlockFlightWithSafeTouchdown() {
        Terrain world = new Terrain();
        PhysicsState state = new PhysicsState(new Vector3d(0, 4, 0), new Vector3d(0, 0, 0.8), false);
        Vector3d target = new Vector3d(0, 0, 400);
        float yaw = 0;
        float pitch = 0;
        int rockets = 30;
        int attachedTicks = 0;
        Set<ElytraFlightController.Phase> phases = new java.util.HashSet<>();
        for (int tick = 0; tick < 2000 && !state.onGround(); tick++) {
            var decision = controller.decide(world, state, yaw, pitch,
                    attachedTicks > 0, target, rockets, 3);
            phases.add(decision.phase());
            if (decision.firework()) {
                assertEquals(0, attachedTicks);
                rockets--;
                attachedTicks = 20;
            }
            yaw = decision.yaw();
            pitch = decision.pitch();
            PhysicsResult result = new GlidingPhysics(yaw, pitch, attachedTicks > 0)
                    .step(world, state, 0.299, 0.6);
            assertFalse(result.collidedX() || result.collidedZ());
            if (result.onGround()) {
                assertTrue(Math.abs(state.velocity().y) <= 0.5);
                assertTrue(Math.hypot(state.velocity().x, state.velocity().z) <= 0.8);
            }
            state = new PhysicsState(result.position(), result.velocity(), result.onGround());
            attachedTicks = Math.max(0, attachedTicks - 1);
        }
        assertTrue(state.onGround(), "Flight did not finish: " + state);
        assertTrue(Math.hypot(state.position().x - target.x, state.position().z - target.z) <= 3);
        assertEquals(0, state.position().y, 0.001);
        assertTrue(phases.contains(ElytraFlightController.Phase.CLIMB));
        assertTrue(phases.contains(ElytraFlightController.Phase.CRUISE));
        assertTrue(phases.contains(ElytraFlightController.Phase.DESCENT));
        assertTrue(phases.contains(ElytraFlightController.Phase.LANDING));
    }

    @Test
    void distantLoadedRidgeSetsClimbAltitude() {
        Terrain ridge = new Terrain() {
            @Override
            public OptionalDouble findHighestCollisionTopExcluding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ, Set<Vector3i> excluded) {
                if (maxZ > 200 && minZ < 216 && minY <= 80 && maxY >= 80) {
                    return OptionalDouble.of(80);
                }
                return super.findHighestCollisionTopExcluding(minX, minY, minZ, maxX, maxY, maxZ, excluded);
            }
        };
        var decision = controller.decide(ridge, airborne(), 0, 0, false,
                new Vector3d(0, 0, 400), 5, 3);
        assertEquals(ElytraFlightController.Phase.CLIMB, decision.phase());
        assertTrue(decision.firework());
        assertTrue(decision.pitch() < 0);
    }

    @Test
    void unknownRouteClimbsConservativelyAndContinues() {
        Terrain unknown = new Terrain() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return chunkZ < 4; }
        };
        var decision = controller.decide(unknown, airborne(), 0, 0, false,
                new Vector3d(0, 0, 400), 5, 3);
        assertEquals(ElytraFlightController.Phase.CLIMB, decision.phase());
        PhysicsResult result = new GlidingPhysics(decision.yaw(), decision.pitch(), decision.firework())
                .step(unknown, airborne(), 0.299, 0.6);
        assertTrue(result.position().z > 0);
    }

    @Test
    void sufficientAltitudeAndSpeedDoNotSpendRockets() {
        var decision = controller.decide(new Terrain(), airborne(), 0, 0, false,
                new Vector3d(0, 0, 400), 5, 3);
        assertFalse(decision.firework());
        assertEquals(ElytraFlightController.Phase.CRUISE, decision.phase());
    }

    @Test
    void rejectsAllUnsafeCandidatesRatherThanSelectingFiniteCollisionPenalty() {
        Terrain blocked = new Terrain() {
            @Override
            public boolean isBoxColliding(double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ) { return true; }
        };
        assertThrows(IllegalStateException.class, () -> controller.decide(blocked, airborne(),
                0, 0, false, new Vector3d(0, 0, 400), 5, 3));
    }

    @Test
    void unknownBoundaryRejectsCrossingButAllowsTurning() {
        Terrain world = new Terrain() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return chunkZ < 1; }
        };
        for (double altitude : new double[] {20, 160}) {
            PhysicsState state = new PhysicsState(new Vector3d(0, altitude, 6),
                    new Vector3d(0, 0, 0.8), false);
            var decision = controller.decide(world, state, 0, 0, false, new Vector3d(0, 0, 400), 5, 3);
            for (int tick = 0; tick < 24; tick++) {
                PhysicsResult result = new GlidingPhysics(decision.yaw(), decision.pitch(), decision.firework())
                        .step(world, state, 0.299, 0.6);
                assertTrue(result.position().z + 0.299 < 16, "Unknown space is unsafe even above the reserve");
                state = new PhysicsState(result.position(), result.velocity(), result.onGround());
            }
        }
    }

    @Test
    void entirelyUnknownSpaceFailsClosedInsteadOfInventingSafeFlight() {
        Terrain unknown = new Terrain() {
            @Override
            public boolean chunkLoaded(int chunkX, int chunkZ) { return false; }
        };
        assertThrows(IllegalStateException.class, () -> controller.decide(unknown, airborne(),
                0, 0, false, new Vector3d(0, 0, 400), 5, 3));
    }

    @Test
    void reportsWarmDecisionLatency() {
        Terrain world = new Terrain();
        record Scenario(ElytraFlightController.Phase phase, double altitude, double distance,
                double speed, int candidates) {}
        Scenario[] scenarios = {
                new Scenario(ElytraFlightController.Phase.CLIMB, 4, 400, 0.8, 80),
                new Scenario(ElytraFlightController.Phase.CRUISE, 20, 400, 0.8, 40),
                new Scenario(ElytraFlightController.Phase.CRUISE, 20, 400, 0.3, 80),
                new Scenario(ElytraFlightController.Phase.DESCENT, 20, 80, 0.8, 40),
                new Scenario(ElytraFlightController.Phase.LANDING, 3, 8, 0.3, 40)
        };
        for (Scenario scenario : scenarios) {
            PhysicsState state = new PhysicsState(new Vector3d(0, scenario.altitude(), 0),
                    new Vector3d(0, 0, scenario.speed()), false);
            Vector3d target = new Vector3d(0, 0, scenario.distance());
            assertEquals(scenario.phase(), controller.decide(world, state, 0, 0, false, target, 30, 3).phase());
            for (int iteration = 0; iteration < 100; iteration++) {
                controller.decide(world, state, 0, 0, false, target, 30, 3);
            }
            long started = System.nanoTime();
            for (int iteration = 0; iteration < 200; iteration++) {
                controller.decide(world, state, 0, 0, false, target, 30, 3);
            }
            // Report measurements, not a machine-dependent timing assertion.
            System.err.printf("Flight decision %s (%d candidates, speed %.1f) warm mean: %.3f ms%n",
                    scenario.phase(), scenario.candidates(), scenario.speed(),
                    (System.nanoTime() - started) / 200_000_000.0);
        }
    }

    @Test
    void cancellingHighAltitudeDescentLeavesSafeUnpoweredTouchdown() {
        Terrain world = new Terrain();
        PhysicsState state = new PhysicsState(new Vector3d(0, 260, 0), new Vector3d(), false);
        Vector3d target = new Vector3d(400, 0, 0);
        float yaw = -90;
        float pitch = 0;
        for (int tick = 0; tick < 160; tick++) {
            var decision = controller.decide(world, state, yaw, pitch, false, target, 64, 16);
            yaw = decision.yaw();
            pitch = decision.pitch();
            PhysicsResult result = new GlidingPhysics(yaw, pitch, false).step(world, state, 0.299, 0.6);
            state = new PhysicsState(result.position(), result.velocity(), result.onGround());
        }
        assertFalse(state.onGround(), "Cancellation must be tested while still airborne");
        for (int tick = 0; tick < 10000 && !state.onGround(); tick++) {
            PhysicsResult result = new GlidingPhysics(yaw, pitch, false).step(world, state, 0.299, 0.6);
            if (result.onGround()) assertTrue(state.velocity().y > -0.5, "Frozen steering must not cause a fatal dive");
            state = new PhysicsState(result.position(), result.velocity(), result.onGround());
        }
        assertTrue(state.onGround());
    }

    @Test
    void rejectsInvalidControlInputs() {
        assertThrows(IllegalArgumentException.class, () -> controller.decide(new Terrain(), airborne(),
                Float.NaN, 0, false, new Vector3d(), 0, 3));
        assertThrows(IllegalArgumentException.class, () -> controller.decide(new Terrain(), airborne(),
                0, 0, false, new Vector3d(), 0, 0));
        assertThrows(IllegalArgumentException.class, () -> controller.decide(new Terrain(), airborne(),
                0, 0, false, new Vector3d(), -1, 3));
    }
}
