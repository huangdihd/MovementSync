package xin.bbtt.flight;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.ElytraFlightController;
import xin.bbtt.physics.FireworkSchedule;
import xin.bbtt.physics.HazardRecovery;
import xin.bbtt.physics.LowAltitudePlanner;
import xin.bbtt.physics.MovementParameters;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class RocketEpisodeSearchTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(),
            "block", 1.5, true, "STONE");
    private static final World RIDGE = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d position) {
            return position.y < 64 || position.x >= 112 && position.x < 128 && position.y < 84
                    ? STONE : super.getBlockStateAt(position);
        }
    };

    @Test void lowLaunchProducesCompleteLifetimeEpisode() {
        var initial = state(16.64, 64.73, 8.5, .04836, -.1210, -.000004636);
        var routeSource = state(16.51, 65.17, 8.5, .01770, -.09189, -.000001697);
        checkEpisode(initial, routeSource, new Vector3d(58.56, 64, 8.5),
                new Vector3d(47.97, 64, 8.5), -90, 0);
    }

    @Test void ridgeApproachProducesCompleteLifetimeEpisode() {
        var initial = state(88.85, 68.20, 7.973, .5898, -.05685, .06370);
        var routeSource = state(84.01, 68.98, 7.512, .59, -.06, .06);
        checkEpisode(initial, routeSource, new Vector3d(109.3, 64, 8.5),
                new Vector3d(98.9, 64, 8.5), -87.76352f, -45);
    }

    @Test void rejectionRecoveryRetainsPoweredMomentumForRidgeCrossing() {
        var source = new PhysicsState(new Vector3d(85.5325606919132, 69.63476925125734, 7.866734048215825),
                new Vector3d(.511367924349455, -.116513729425292, .012493220003604),
                false, 1.116513729425293, PlayerPose.GLIDING);
        var live = new PhysicsState(new Vector3d(88.19373933075155, 68.99571338899426, 8.012235179255878),
                new Vector3d(.547049219002832, -.133906327919651, .038614775707906),
                false, 1.133906327919647, PlayerPose.GLIDING);
        var routeSource = state(82.55, 70.24, 7.909, .51, -.11, .01);
        var route = LowAltitudePlanner.planPursuit(RIDGE, routeSource,
                new Vector3d(109.3, 64, 8.5), MovementParameters.DEFAULT);
        assertNotNull(route);
        var goal = new Vector3d(97.3, 64, 8.5);
        var selected = new ElytraFlightController.Decision(272.7364f, -45, true,
                ElytraFlightController.Phase.CRUISE);
        var episode = LocalGotoController.prepareEpisode(RIDGE, MovementParameters.DEFAULT,
                source, selected, true, true, 5, FireworkSchedule.lifetimeBounds(1),
                goal, false, false, route);
        assertEquals(27, episode.prefix().size());
        assertEquals(6, episode.reserve().size());
        assertEquals(1, episode.bridge().size());
        for (var initial : java.util.List.of(source, live)) {
            var proof = LocalGotoController.rocketTransitionProof(RIDGE, MovementParameters.DEFAULT,
                    initial, episode.prefix(), episode.reserve(), episode.bridge(), goal,
                    new HazardRecovery.Context(new int[] {1}, 0, false, null), route);
            assertNotNull(proof, "fixed controls must survive publication-state drift");
            assertEquals(25, proof.branches().size());
            for (var branch : proof.branches().subList(1, proof.branches().size())) {
                var state = branch.states().get(branch.states().size() - 1);
                for (int tick = 0; tick < 30; tick++) {
                    var next = new xin.bbtt.physics.GlidingPhysics(selected.yaw(), 0, false)
                            .step(RIDGE, state, PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                    assertFalse(next.onGround() || next.collidedX() || next.collidedZ(),
                            "a complete episode must leave enough momentum to clear the ridge: " + state);
                    state = new PhysicsState(next.position(), next.velocity(), false,
                            next.fallDistance(), PlayerPose.GLIDING);
                }
                assertTrue(state.position().x > 128 + PlayerPose.HALF_WIDTH,
                        "all legal lifetimes and acknowledgement delays must cross: " + state);
            }
        }
    }

    @Test void roofedStaticLaunchProducesCompleteLifetimeEpisode() {
        World roofed = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 64 || position.y >= 80 ? STONE : super.getBlockStateAt(position);
            }
        };
        var initial = state(16.5, 65.17, 8.509, 0, -.08418, .008504);
        var selected = new ElytraFlightController.Decision(0, -15, true,
                ElytraFlightController.Phase.CRUISE);
        var goal = new Vector3d(16.5, 81.25, 32.509);
        var route = new xin.bbtt.physics.WaypointFlightTarget(java.util.List.of(initial.position(), goal),
                goal, false, false, 4);
        var episode = LocalGotoController.prepareEpisode(roofed, MovementParameters.DEFAULT,
                initial, selected, true, true, 5, FireworkSchedule.lifetimeBounds(1),
                goal, false, false, route);
        assertEquals(27, episode.prefix().size());
        assertNotNull(LocalGotoController.rocketTransitionProof(roofed, MovementParameters.DEFAULT,
                initial, episode.prefix(), episode.reserve(), episode.bridge(), goal,
                new HazardRecovery.Context(new int[] {1}, 0, false, null), route));
    }

    @Test void shoreLaunchProducesCompleteLifetimeEpisodeBeforeLosingTakeoffHeight() {
        var lava = new BlockState("minecraft:lava", 0, Map.of(), "empty", 0, false, "LAVA");
        World sea = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return x >= -2 && x <= 12 && z >= -2 && z <= 2; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                if (position.y >= 60 || position.y < 29
                        || position.x >= 0 && position.x < 16 && Math.abs(position.z) < 8 && position.y < 32
                        || position.x >= 128 && position.y < 42) return STONE;
                return position.y < 32 ? lava : super.getBlockStateAt(position);
            }
        };
        var initial = state(8.509, 33.17, .5, .008504, -.08418, 0);
        var goal = new Vector3d(144.5, 42, .5);
        var route = LowAltitudePlanner.plan(sea, initial, goal, MovementParameters.DEFAULT);
        assertNotNull(route);
        var selected = new ElytraFlightController.Decision(-90, -15, true,
                ElytraFlightController.Phase.CRUISE);
        var episode = LocalGotoController.prepareEpisode(sea, MovementParameters.DEFAULT,
                initial, selected, true, true, 5, FireworkSchedule.lifetimeBounds(1),
                goal, false, false, route);
        assertEquals(27, episode.prefix().size());
        assertNotNull(LocalGotoController.rocketTransitionProof(sea, MovementParameters.DEFAULT,
                initial, episode.prefix(), episode.reserve(), episode.bridge(), goal,
                new HazardRecovery.Context(new int[] {1}, 0, false, null), route));
    }

    private static void checkEpisode(PhysicsState initial, PhysicsState routeSource,
            Vector3d routeGoal, Vector3d trackingGoal, float yaw, float pitch) {
        var route = LowAltitudePlanner.planPursuit(RIDGE, routeSource, routeGoal,
                MovementParameters.DEFAULT);
        assertNotNull(route);
        var selected = new ElytraFlightController.Decision(yaw, pitch, true,
                ElytraFlightController.Phase.CRUISE);
        var episode = LocalGotoController.prepareEpisode(RIDGE, MovementParameters.DEFAULT,
                initial, selected, true, true, 5, FireworkSchedule.lifetimeBounds(1),
                trackingGoal, false, false, route);
        assertEquals(27, episode.prefix().size());
        assertEquals(6, episode.reserve().size());
        assertEquals(1, episode.bridge().size());
        assertNotNull(LocalGotoController.rocketTransitionProof(RIDGE, MovementParameters.DEFAULT,
                initial, episode.prefix(), episode.reserve(), episode.bridge(), trackingGoal,
                new HazardRecovery.Context(new int[] {1}, 0, false, null), route));
    }

    private static PhysicsState state(double x, double y, double z,
            double velocityX, double velocityY, double velocityZ) {
        return new PhysicsState(new Vector3d(x, y, z), new Vector3d(velocityX, velocityY, velocityZ),
                false, 1, PlayerPose.GLIDING);
    }
}
