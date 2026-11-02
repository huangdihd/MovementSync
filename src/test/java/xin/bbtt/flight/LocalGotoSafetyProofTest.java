package xin.bbtt.flight;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.ElytraFlightController;
import xin.bbtt.physics.FireworkSchedule;
import xin.bbtt.physics.FlightControlAdmission;
import xin.bbtt.physics.FlightHazards;
import xin.bbtt.physics.GlidingPhysics;
import xin.bbtt.physics.HazardRecovery;
import xin.bbtt.physics.LocalAirRoute;
import xin.bbtt.physics.MovementParameters;
import xin.bbtt.physics.PhysicsResult;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerInput;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.physics.WaypointFlightTarget;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class LocalGotoSafetyProofTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(),
            "block", 1.5, true, "STONE");
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0, Map.of(),
            "empty", 0, false, "");
    private static final World AIR = new World() {
        @Override public boolean chunkLoaded(int x, int z) { return true; }
    };

    @Test void poweredSteepClimbRetainsWorkerProvenCommitmentAndBoundedCruiseReserve() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .4),
                false, 0, PlayerPose.GLIDING);
        var selected = new ElytraFlightController.Decision(71, -60, false,
                ElytraFlightController.Phase.CRUISE);
        var goal = new Vector3d(-20, 110, 8);

        var episode = LocalGotoController.prepareEpisode(AIR, MovementParameters.DEFAULT,
                initial, selected, true, true, 12, goal, false, false, null);

        assertEquals(12, episode.prefix().size(),
                "the full worker-proven climb must not be truncated before its fixed reserve");
        assertEquals(6, episode.reserve().size());
        assertEquals(-60f, episode.prefix().get(0).control().pitch());
        assertTrue(episode.prefix().get(0).control().firework());
        assertTrue(episode.reserve().stream().allMatch(step ->
                step.control().pitch() == -20f && !step.control().firework()));
        var proof = LocalGotoController.attachmentExpiryProof(AIR, MovementParameters.DEFAULT,
                initial, episode.prefix(), episode.reserve(), episode.bridge(), goal, false, null);
        assertNotNull(proof);
        assertEquals(episode.prefix().size() + episode.reserve().size()
                        + episode.bridge().size() + 1,
                proof.branches().size());
    }

    @Test void boundedPoweredCruiseRetainsItsFullCommitment() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .4),
                false, 0, PlayerPose.GLIDING);
        var selected = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);

        var episode = LocalGotoController.prepareEpisode(AIR, MovementParameters.DEFAULT,
                initial, selected, true, false, 12, new Vector3d(0, 100, 40),
                false, false, null);

        assertEquals(12, episode.prefix().size(),
                "a fully proven powered cruise retains its complete commitment");
        assertEquals(6, episode.reserve().size());
    }

    @Test void poweredDescentReserveDoesNotReverseTheProvenVerticalIntent() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .4),
                false, 0, PlayerPose.GLIDING);
        var selected = new ElytraFlightController.Decision(0, 20, false,
                ElytraFlightController.Phase.CRUISE);

        var episode = LocalGotoController.prepareEpisode(AIR, MovementParameters.DEFAULT,
                initial, selected, true, false, 1, new Vector3d(0, 90, 20),
                false, false, null);

        assertEquals(1, episode.prefix().size());
        assertEquals(6, episode.reserve().size());
        assertTrue(episode.reserve().stream().allMatch(step ->
                step.control().pitch() == 20f && !step.control().firework()),
                "a descending powered commitment must not become a climb while thrust remains attached");
    }

    @Test void fallMarginIncludesTheFinalReserveAndProofOnlyBridgeTransitions() {
        var goal = new Vector3d(0, 100, 40);
        for (boolean powered : new boolean[] {false, true}) {
            for (int crossingTick : new int[] {7, 8}) {
                double verticalSpeed = powered ? 0 : -.2;
                if (crossingTick == 7) verticalSpeed = -.3;
                var initial = new PhysicsState(new Vector3d(0, 100, 0),
                        new Vector3d(0, verticalSpeed, .4), false, 0, PlayerPose.GLIDING);
                var control = new ElytraFlightController.Decision(0, powered ? 20 : 70, false,
                        ElytraFlightController.Phase.CRUISE);
                var states = replay(initial, control, crossingTick, powered ? crossingTick : 0);
                assertFallThresholdCrossesOnLastTransition(states);

                var ordinary = LocalGotoController.prepareEpisode(AIR, MovementParameters.DEFAULT,
                        initial, control, powered, false, 1, null, goal, false, false, null, false);

                assertEquals(1, ordinary.prefix().size());
                assertEquals(6, ordinary.reserve().size());
                assertEquals(1, ordinary.bridge().size());
                assertEquals(states.get(crossingTick - 1), crossingTick == 7
                        ? ordinary.reserve().get(5).expected() : ordinary.bridge().get(0).expected(),
                        "the stored pre-state is safe; the transition result exhausts the margin");
                var strict = LocalGotoController.prepareEpisode(AIR, MovementParameters.DEFAULT,
                        initial, control, powered, false, 1, null, goal, false, false, null, true);

                assertTrue(strict.prefix().isEmpty(),
                        "a safe prefix cannot borrow fall margin from its reserve or bridge");
                assertTrue(strict.reserve().isEmpty());
                assertTrue(strict.bridge().isEmpty());
            }
        }
    }

    @Test void rejectedRocketMustPreserveFallMarginThroughTheFinalBridgeTransition() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, -.8, .4),
                false, 1.2, PlayerPose.GLIDING);
        var control = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);
        var steps = repeatedSteps(initial, control, 3);
        var rejected = replay(initial, control, 3, 0);
        assertFallThresholdCrossesOnLastTransition(rejected);
        var lifetime = FireworkSchedule.lifetimeBounds(0);
        for (int poweredTicks = lifetime.minimumPoweredTicks();
                poweredTicks <= lifetime.maximumPoweredTicks(); poweredTicks++) {
            for (int acknowledgementDelay = 0; acknowledgementDelay <= 1; acknowledgementDelay++) {
                assertTrue(replay(initial, control, 3, acknowledgementDelay, poweredTicks).stream()
                        .allMatch(state -> state.fallDistance() < MovementParameters.DEFAULT.safeFallDistance()),
                        "only the rejected request exhausts this fixture's fall margin");
            }
        }
        var resources = new HazardRecovery.Context(new int[] {0}, 0, false, null);
        var goal = new Vector3d(0, 100, 30);
        var proof = LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 1), steps.subList(1, 2), steps.subList(2, 3),
                goal, resources, null, false);

        assertNotNull(proof);
        assertTrue(proof.branches().stream().anyMatch(branch -> branch.states().equals(rejected)));
        assertNull(LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 1), steps.subList(1, 2), steps.subList(2, 3),
                goal, resources, null, true));
        assertNotNull(LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 1), steps.subList(1, 2), List.of(),
                goal, resources, null, true), "the same executable controls are safe without the bridge");
    }

    @Test void everyPoweredRocketLifetimeMustPreserveFallMargin() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .4),
                false, 0, PlayerPose.GLIDING);
        var control = new ElytraFlightController.Decision(0, 60, false,
                ElytraFlightController.Phase.CRUISE);
        var steps = repeatedSteps(initial, control, 4);
        var resources = new HazardRecovery.Context(new int[] {0}, 0, false, null);
        var goal = new Vector3d(0, 100, 30);
        var proof = LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 1), steps.subList(1, 3), steps.subList(3, 4),
                goal, resources, null, false);

        assertNotNull(proof);
        assertTrue(replay(initial, control, 4, 0).stream()
                .allMatch(state -> state.fallDistance() < MovementParameters.DEFAULT.safeFallDistance()),
                "rejected thrust must not mask the powered-branch rejection");
        var lifetime = FireworkSchedule.lifetimeBounds(0);
        for (int poweredTicks = lifetime.minimumPoweredTicks();
                poweredTicks <= lifetime.maximumPoweredTicks(); poweredTicks++) {
            var states = replay(initial, control, 4, poweredTicks);
            assertFallThresholdCrossesOnLastTransition(states);
            assertTrue(proof.branches().stream().anyMatch(branch -> branch.states().equals(states)));
        }
        assertNull(LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 1), steps.subList(1, 3), steps.subList(3, 4),
                goal, resources, null, true));
        assertNotNull(LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 1), steps.subList(1, 3), List.of(),
                goal, resources, null, true));
    }

    @Test void delayedAcknowledgementCannotHideAPoweredFallMarginViolation() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, -.3, .2),
                false, 0, PlayerPose.GLIDING);
        var climb = new ElytraFlightController.Decision(0, -80, false,
                ElytraFlightController.Phase.CRUISE);
        var dive = new ElytraFlightController.Decision(0, 80, false,
                ElytraFlightController.Phase.CRUISE);
        var steps = new ArrayList<>(repeatedSteps(initial, climb, 1));
        steps.addAll(repeatedSteps(initial, dive, 3));
        var resources = new HazardRecovery.Context(new int[] {0}, 0, false, null);
        var goal = new Vector3d(0, 100, 30);
        var proof = LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 1), steps.subList(1, 3), steps.subList(3, 4),
                goal, resources, null, false);

        assertNotNull(proof);
        assertTrue(replay(initial, steps, 0, 0).stream()
                .allMatch(state -> state.fallDistance() < MovementParameters.DEFAULT.safeFallDistance()));
        var lifetime = FireworkSchedule.lifetimeBounds(0);
        for (int poweredTicks = lifetime.minimumPoweredTicks();
                poweredTicks <= lifetime.maximumPoweredTicks(); poweredTicks++) {
            assertTrue(replay(initial, steps, 0, poweredTicks).stream()
                    .allMatch(state -> state.fallDistance() < MovementParameters.DEFAULT.safeFallDistance()),
                    "every immediate acknowledgement remains below the threshold");
            var delayed = replay(initial, steps, 1, poweredTicks);
            assertFallThresholdCrossesOnLastTransition(delayed);
            assertTrue(proof.branches().stream().anyMatch(branch -> branch.states().equals(delayed)));
        }
        assertNull(LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 1), steps.subList(1, 3), steps.subList(3, 4),
                goal, resources, null, true));
        assertNotNull(LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 1), steps.subList(1, 3), List.of(),
                goal, resources, null, true));
    }

    @Test void earlyRocketExpiryMustPreserveFallMarginAfterThrustEnds() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .4),
                false, 0, PlayerPose.GLIDING);
        var cruise = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);
        var dive = new ElytraFlightController.Decision(0, 40, false,
                ElytraFlightController.Phase.CRUISE);
        var climb = new ElytraFlightController.Decision(0, -80, false,
                ElytraFlightController.Phase.CRUISE);
        var steps = new ArrayList<>(repeatedSteps(initial, cruise, 7));
        steps.addAll(repeatedSteps(initial, dive, 4));
        steps.addAll(repeatedSteps(initial, climb, 2));
        var lifetime = FireworkSchedule.lifetimeBounds(0);
        var expired = replay(initial, steps, 0, lifetime.minimumPoweredTicks());
        assertFallThresholdCrossesOnLastTransition(expired);
        assertTrue(steps.size() - 1 > lifetime.minimumPoweredTicks(),
                "the violating bridge transition must occur after minimum-lifetime expiry");
        assertTrue(replay(initial, steps, 0, 0).stream()
                .allMatch(state -> state.fallDistance() < MovementParameters.DEFAULT.safeFallDistance()));
        for (int acknowledgementDelay = 0; acknowledgementDelay <= 1; acknowledgementDelay++) {
            assertTrue(replay(initial, steps, acknowledgementDelay, lifetime.maximumPoweredTicks()).stream()
                    .allMatch(state -> state.fallDistance() < MovementParameters.DEFAULT.safeFallDistance()),
                    "neither maximum-lifetime branch exhausts the margin");
        }
        var resources = new HazardRecovery.Context(new int[] {0}, 0, false, null);
        var goal = new Vector3d(0, 100, 30);
        var proof = LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 7), steps.subList(7, 12), steps.subList(12, 13),
                goal, resources, null, false);

        assertNotNull(proof);
        assertTrue(proof.branches().stream().anyMatch(branch -> branch.states().equals(expired)));
        assertNull(LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 7), steps.subList(7, 12), steps.subList(12, 13),
                goal, resources, null, true));
        assertNotNull(LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 7), steps.subList(7, 12), List.of(),
                goal, resources, null, true));
    }

    @Test void attachmentExpiryFallMarginIncludesTheFinalBridgeTransition() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, -.8, .4),
                false, 1.2, PlayerPose.GLIDING);
        var control = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);
        var steps = repeatedSteps(initial, control, 3);
        var expired = replay(initial, control, 3, 0);
        assertFallThresholdCrossesOnLastTransition(expired);
        for (int expiryTick = 1; expiryTick <= 3; expiryTick++) {
            assertTrue(replay(initial, control, 3, expiryTick).stream()
                    .allMatch(state -> state.fallDistance() < MovementParameters.DEFAULT.safeFallDistance()),
                    "only expiry before the first tick exhausts the fall margin");
        }
        var goal = new Vector3d(0, 100, 30);
        var proof = LocalGotoController.attachmentExpiryProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 1), steps.subList(1, 2), steps.subList(2, 3),
                goal, false, null);

        assertNotNull(proof);
        assertEquals(expired, proof.branches().get(0).states());
        assertNull(LocalGotoController.attachmentExpiryProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 1), steps.subList(1, 2), steps.subList(2, 3),
                goal, true, null));
        assertNotNull(LocalGotoController.attachmentExpiryProof(AIR, MovementParameters.DEFAULT,
                initial, steps.subList(0, 1), steps.subList(1, 2), List.of(), goal, true, null));
    }

    @Test void attachmentExpiryProofContainsEveryPoweredPrefixLength() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .4),
                false, 0, PlayerPose.GLIDING);
        var control = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);
        var prefix = repeatedSteps(initial, control, 3);
        var reserve = repeatedSteps(initial, control, 2);

        var proof = LocalGotoController.attachmentExpiryProof(AIR, MovementParameters.DEFAULT,
                initial, prefix, reserve, new Vector3d(0, 100, 20), false, null);

        assertNotNull(proof);
        assertEquals(6, proof.branches().size());
        for (int expiryTick = 0; expiryTick <= 5; expiryTick++) {
            var states = proof.branches().get(expiryTick).states();
            assertEquals(6, states.size());
            PhysicsState state = initial;
            for (int tick = 0; tick < 5; tick++) {
                var result = new GlidingPhysics(control.yaw(), control.pitch(), tick < expiryTick,
                        PlayerInput.NONE, MovementParameters.DEFAULT).step(AIR, state,
                                PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                state = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                        result.fallDistance(), PlayerPose.GLIDING);
                assertEquals(state, states.get(tick + 1));
                assertTrue(proof.corridor().contains(states.get(tick).position(), state.position()));
            }
        }
    }

    @Test void anyUnsafeExpiryBranchRejectsTheWholeProof() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, .05, .2),
                false, 0, PlayerPose.GLIDING);
        var control = new ElytraFlightController.Decision(0, -80, false,
                ElytraFlightController.Phase.CRUISE);
        var steps = repeatedSteps(initial, control, 4);
        var unpoweredPath = replay(initial, control, 4, 0).stream()
                .map(PhysicsState::position).toList();
        var unpoweredOnly = xin.bbtt.physics.WaypointFlightTarget.executionWitness(
                List.of(unpoweredPath), new Vector3d(0, 100, 20), false);

        assertNull(LocalGotoController.attachmentExpiryProof(AIR, MovementParameters.DEFAULT,
                initial, steps, List.of(), new Vector3d(0, 100, 20), false, unpoweredOnly));
    }

    @Test void dangerousEntryTimelineIncludesReserveAndEpisodeEndTicks() {
        var states = List.of(state(0), state(1), state(2), state(3), state(4), state(5));

        var reserveEntry = LocalGotoController.firstDangerousEntry(coast(4), states);
        assertNotNull(reserveEntry);
        assertEquals(4, reserveEntry.elapsedTicks());
        assertEquals(state(4), reserveEntry.state());

        var episodeEntry = LocalGotoController.firstDangerousEntry(coast(5), states);
        assertNotNull(episodeEntry);
        assertEquals(5, episodeEntry.elapsedTicks());
    }

    @Test void discontinuousPrefixReserveTimelineFailsClosed() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .4),
                false, 0, PlayerPose.GLIDING);
        var control = new ElytraFlightController.Decision(0, 0, false,
                ElytraFlightController.Phase.CRUISE);
        var prefix = List.of(new FlightControlAdmission.Step(control, initial));
        var reserve = List.of(new FlightControlAdmission.Step(control, state(20)));

        var entry = LocalGotoController.firstDangerousEntry(AIR, MovementParameters.DEFAULT,
                initial, prefix, reserve, state(21));

        assertNotNull(entry);
        assertFalse(entry.exactTick());
    }

    @Test void transientExposureBetweenSafeTickBoundariesFailsClosed() {
        var states = List.of(state(.5), state(2.5));
        World strip = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                if (position.y < 32) return (int) Math.floor(position.x) == 1 ? LAVA : STONE;
                return super.getBlockStateAt(position);
            }
        };

        var entry = LocalGotoController.firstDangerousEntry(strip, states);

        assertNotNull(entry);
        assertFalse(entry.exactTick());
    }

    @Test void dangerousEntryResourcesUseActualElapsedTickAndDoNotCreditDelayedRocket() {
        var context = new HazardRecovery.Context(new int[] {1, 2}, 0, false, null);
        var entry = new LocalGotoController.DangerousEntry(state(4), 4);

        var remaining = LocalGotoController.resourcesAfterDangerousEntry(context, entry, true);

        assertArrayEquals(new int[] {2}, remaining.rocketFlightDurations());
        assertEquals(36, remaining.cooldownTicks());
        assertFalse(remaining.attached());
        assertEquals(0, remaining.guaranteedBoostTicks());
    }

    @Test void rocketTransitionProofUsesRequestRelativeBoundedLifetime() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .4),
                false, 0, PlayerPose.GLIDING);
        var control = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);
        var steps = repeatedSteps(initial, control, 24);
        var resources = new HazardRecovery.Context(new int[] {0}, 0, false, null);

        var proof = LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps, List.of(), new Vector3d(0, 100, 30), resources, null);

        assertNotNull(proof);
        assertEquals(25, proof.branches().size(),
                "rejection plus immediate and next-tick observation of every legal flight=0 lifetime");
        assertTrue(proof.branches().stream().anyMatch(branch -> branch.states().equals(
                replay(initial, control, 24, 0))), "a rejected request has no thrust");
        assertTrue(proof.branches().stream().anyMatch(branch -> branch.states().equals(
                replay(initial, control, 24, 11))), "flight=0 minimum expiry is tick 11");
        assertTrue(proof.branches().stream().anyMatch(branch -> branch.states().equals(
                replay(initial, control, 24, 22))), "flight=0 maximum expiry is tick 22");
        assertTrue(proof.branches().stream().anyMatch(branch -> branch.states().equals(
                replay(initial, control, 24, 1, 11))),
                "next-tick observation still expires at request-relative tick 11");
        assertFalse(proof.branches().stream().anyMatch(branch -> branch.states().equals(
                replay(initial, control, 24, 23))), "physically impossible overlong thrust is excluded");
        assertFalse(proof.branches().stream().anyMatch(branch -> branch.states().equals(
                replay(initial, control, 24, 10, 11))),
                "late acknowledgement must not create delayed physical thrust");
    }

    @Test void delayedRocketBranchLeavingPublishedCorridorRejectsProof() {
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .4),
                false, 0, PlayerPose.GLIDING);
        var control = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);
        var steps = repeatedSteps(initial, control, 4);
        var poweredPath = replay(initial, control, 4, 4).stream()
                .map(PhysicsState::position).toList();
        var poweredOnly = xin.bbtt.physics.WaypointFlightTarget.executionWitness(
                List.of(poweredPath), new Vector3d(0, 100, 30), false);

        assertNull(LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, steps, List.of(), new Vector3d(0, 100, 30),
                new HazardRecovery.Context(new int[] {0}, 0, false, null), poweredOnly));
    }

    @Test void rejectedRocketMayLandOutsidePoweredCorridorInPrefixOrReserve() {
        World ground = landingWorld(STONE);
        var fixture = rocketLandingFixture(ground, landingInitialState());
        var rejected = rejectedLanding(ground, fixture);
        var touchdown = rejected.get(rejected.size() - 1);
        var before = rejected.get(rejected.size() - 2);
        int landingTicks = rejected.size() - 1;

        assertTrue(landingTicks > 1 && landingTicks < fixture.steps().size());
        assertTrue(LocalAirRoute.visible(ground, before.position(), touchdown.position(),
                PlayerPose.GLIDING), "the touchdown sweep must be loaded and collision-free");
        assertTrue(FlightHazards.safeLanding(ground, touchdown.position()));
        assertTrue(before.fallDistance() + before.position().y - touchdown.position().y
                < MovementParameters.DEFAULT.safeFallDistance());

        for (int prefixTicks : new int[] {landingTicks, 1}) {
            var proof = LocalGotoController.rocketTransitionProof(ground, MovementParameters.DEFAULT,
                    fixture.initial(), fixture.steps().subList(0, prefixTicks),
                    fixture.steps().subList(prefixTicks, fixture.steps().size()),
                    fixture.corridor().goal(), rocketResources(), fixture.corridor());

            assertNotNull(proof, "safe rejection may terminate in either prefix or reserve");
            assertTrue(proof.branches().stream().anyMatch(branch -> branch.states().equals(rejected)),
                    "the proof must retain the actual off-corridor rejected landing branch");
            assertTrue(proof.corridor().sameCorridor(fixture.corridor()),
                    "accepting rejection must not enlarge the published powered corridor");
            var powered = proof.branches().stream()
                    .filter(branch -> !branch.states().equals(rejected)).toList();
            var lifetime = FireworkSchedule.lifetimeBounds(1);
            assertEquals(2 * (lifetime.maximumPoweredTicks() - lifetime.minimumPoweredTicks() + 1),
                    powered.size());
            for (var branch : powered) {
                assertEquals(fixture.steps().size() + 1, branch.states().size(),
                        "rejected touchdown must not truncate any possible powered branch");
                assertTrue(branch.states().stream().noneMatch(PhysicsState::onGround));
                for (int tick = 1; tick < branch.states().size(); tick++) {
                    assertTrue(fixture.corridor().contains(branch.states().get(tick - 1).position(),
                            branch.states().get(tick).position()));
                }
            }
        }
    }

    @Test void rejectedRocketCannotBorrowItsLandingTickFromProofOnlyHandoff() {
        World ground = landingWorld(STONE);
        var fixture = rocketLandingFixture(ground, landingInitialState());
        int landingTicks = rejectedLanding(ground, fixture).size() - 1;
        assertTrue(landingTicks > 2);
        var prefix = fixture.steps().subList(0, 1);

        assertNotNull(LocalGotoController.rocketTransitionProof(ground, MovementParameters.DEFAULT,
                fixture.initial(), prefix, fixture.steps().subList(1, landingTicks),
                fixture.steps().subList(landingTicks, fixture.steps().size()),
                fixture.corridor().goal(), rocketResources(), fixture.corridor()),
                "landing on the final executable reserve tick is allowed");
        assertNull(LocalGotoController.rocketTransitionProof(ground, MovementParameters.DEFAULT,
                fixture.initial(), prefix, fixture.steps().subList(1, landingTicks - 1),
                fixture.steps().subList(landingTicks - 1, fixture.steps().size()),
                fixture.corridor().goal(), rocketResources(), fixture.corridor()),
                "the same landing on the first proof-only handoff tick must fail closed");
    }

    @Test void rejectedRocketCannotStayAirborneOutsidePoweredCorridor() {
        var initial = new PhysicsState(new Vector3d(.5, 100, .5), new Vector3d(0, -.1, .3),
                false, 0, PlayerPose.GLIDING);
        var fixture = rocketLandingFixture(AIR, initial);
        var rejected = replay(initial, fixture.control(), fixture.steps().size(), 0);
        boolean departed = false;
        for (int tick = 1; tick < rejected.size(); tick++) {
            departed |= !fixture.corridor().contains(rejected.get(tick - 1).position(),
                    rejected.get(tick).position());
        }

        assertTrue(departed, "the rejected branch must actually leave the powered corridor");
        assertTrue(rejected.stream().noneMatch(PhysicsState::onGround));
        assertNull(LocalGotoController.rocketTransitionProof(AIR, MovementParameters.DEFAULT,
                initial, fixture.steps().subList(0, 1),
                fixture.steps().subList(1, fixture.steps().size()), fixture.corridor().goal(),
                rocketResources(), fixture.corridor()),
                "all powered witnesses are valid, but airborne off-route rejection is not");
    }

    @Test void rejectedRocketCannotUseUnloadedSupportOutsidePoweredCorridor() {
        World unloaded = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return z >= 0; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 1 ? STONE : super.getBlockStateAt(position);
            }
        };
        // Rejection drifts backwards across the chunk edge; both accepted thrust
        // timings turn forwards while their complete body remains in loaded terrain.
        var initial = new PhysicsState(new Vector3d(.5, 1.45, .9), new Vector3d(0, -.1, -.3),
                false, 0, PlayerPose.GLIDING);
        var fixture = rocketLandingFixture(unloaded, initial);
        World ground = landingWorld(STONE);
        var rejected = rejectedLanding(ground, fixture);
        var touchdown = rejected.get(rejected.size() - 1);

        assertNotNull(LocalGotoController.rocketTransitionProof(ground, MovementParameters.DEFAULT,
                initial, fixture.steps().subList(0, 1),
                fixture.steps().subList(1, fixture.steps().size()), fixture.corridor().goal(),
                rocketResources(), fixture.corridor()),
                "the same branch must be admitted when its supporting terrain is loaded");
        assertTrue(FlightHazards.safeLanding(unloaded, touchdown.position()),
                "absence of hot blocks does not prove the supporting chunk is loaded");
        assertFalse(LocalAirRoute.visible(unloaded, rejected.get(rejected.size() - 2).position(),
                touchdown.position(), PlayerPose.GLIDING));
        assertNull(LocalGotoController.rocketTransitionProof(unloaded, MovementParameters.DEFAULT,
                fixture.initial(), fixture.steps().subList(0, 1),
                fixture.steps().subList(1, fixture.steps().size()), fixture.corridor().goal(),
                rocketResources(), fixture.corridor()));
    }

    @Test void rejectedRocketCannotLandOnMagmaOutsidePoweredCorridor() {
        var magma = new BlockState("minecraft:magma_block", 1, Map.of(),
                "block", 1.5, true, "STONE");
        World ground = landingWorld(magma);
        var fixture = rocketLandingFixture(ground, landingInitialState());
        var rejected = rejectedLanding(ground, fixture);

        assertFalse(FlightHazards.safeLanding(ground, rejected.get(rejected.size() - 1).position()));
        assertNull(LocalGotoController.rocketTransitionProof(ground, MovementParameters.DEFAULT,
                fixture.initial(), fixture.steps().subList(0, 1),
                fixture.steps().subList(1, fixture.steps().size()), fixture.corridor().goal(),
                rocketResources(), fixture.corridor()));
    }

    @Test void rejectedRocketCannotEraseFallDamageWhenTouchdownResetsFallDistance() {
        World ground = landingWorld(STONE);
        var initial = new PhysicsState(new Vector3d(.5, 2.3, .5), new Vector3d(0, -.6, .3),
                false, MovementParameters.DEFAULT.safeFallDistance(), PlayerPose.GLIDING);
        var fixture = rocketLandingFixture(ground, initial);
        var rejected = rejectedLanding(ground, fixture);
        var touchdown = rejected.get(rejected.size() - 1);
        var before = rejected.get(rejected.size() - 2);

        assertTrue(FlightHazards.safeLanding(ground, touchdown.position()));
        assertTrue(LocalAirRoute.visible(ground, before.position(), touchdown.position(),
                PlayerPose.GLIDING));
        assertEquals(0, touchdown.fallDistance(), "physics resets the counter on ground contact");
        assertTrue(before.fallDistance() + before.position().y - touchdown.position().y
                >= MovementParameters.DEFAULT.safeFallDistance(),
                "the pre-contact fall must still be rejected despite the reset counter");
        assertNull(LocalGotoController.rocketTransitionProof(ground, MovementParameters.DEFAULT,
                initial, fixture.steps().subList(0, 1),
                fixture.steps().subList(1, fixture.steps().size()), fixture.corridor().goal(),
                rocketResources(), fixture.corridor()));
    }

    @Test void rejectedRocketCannotTreatWallCollisionAsSafeOffCorridorLanding() {
        World wall = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                if (position.y < 1 || Math.floor(position.y) == 1 && Math.floor(position.z) == 4)
                    return STONE;
                return super.getBlockStateAt(position);
            }
        };
        var initial = new PhysicsState(new Vector3d(.5, 2.3, .5), new Vector3d(0, -.1, .3),
                false, 0, PlayerPose.GLIDING);
        var fixture = rocketLandingFixture(wall, initial);
        var rejected = rejectedLanding(wall, fixture);
        var before = rejected.get(rejected.size() - 2);
        var result = new GlidingPhysics(fixture.control().yaw(), fixture.control().pitch(), false,
                PlayerInput.NONE, MovementParameters.DEFAULT).step(wall, before,
                        PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());

        assertTrue(result.onGround());
        assertTrue(result.collidedZ(), "the rejected branch must actually hit the low wall");
        assertTrue(FlightHazards.safeLanding(wall, result.position()),
                "ordinary stone must not make the collision test pass through hazard rejection");
        assertNull(LocalGotoController.rocketTransitionProof(wall, MovementParameters.DEFAULT,
                fixture.initial(), fixture.steps().subList(0, 1),
                fixture.steps().subList(1, fixture.steps().size()), fixture.corridor().goal(),
                rocketResources(), fixture.corridor()));
    }

    @Test void rocketGroundContactRejectsTheCompleteEpisode() {
        World ground = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return Math.floor(position.y) == 0 ? STONE : super.getBlockStateAt(position);
            }
        };
        var initial = new PhysicsState(new Vector3d(.5, 1.02, .5),
                new Vector3d(0, -.2, .1), false, 0, PlayerPose.GLIDING);
        var control = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);

        assertNull(LocalGotoController.rocketTransitionProof(ground, MovementParameters.DEFAULT,
                initial, repeatedSteps(initial, control, 2), List.of(), new Vector3d(.5, 1, 8),
                new HazardRecovery.Context(new int[] {0}, 0, false, null), null));
    }

    @Test void hazardousRocketBranchRejectsProof() {
        World fire = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return new BlockState("minecraft:fire", 0, Map.of(), "empty", 0, false, "");
            }
        };
        var initial = new PhysicsState(new Vector3d(0, 100, 0), new Vector3d(0, 0, .4),
                false, 0, PlayerPose.GLIDING);
        var control = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);

        assertNull(LocalGotoController.rocketTransitionProof(fire, MovementParameters.DEFAULT,
                initial, repeatedSteps(initial, control, 2), List.of(), new Vector3d(0, 100, 30),
                new HazardRecovery.Context(new int[] {0}, 0, false, null), null));
    }

    private record RocketLandingFixture(PhysicsState initial,
            ElytraFlightController.Decision control, List<FlightControlAdmission.Step> steps,
            WaypointFlightTarget corridor) {}

    private static PhysicsState landingInitialState() {
        return new PhysicsState(new Vector3d(.5, 1.45, .5), new Vector3d(0, -.1, .3),
                false, 0, PlayerPose.GLIDING);
    }

    private static HazardRecovery.Context rocketResources() {
        return new HazardRecovery.Context(new int[] {1}, 0, false, null);
    }

    private static World landingWorld(BlockState support) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                return position.y < 1 ? support : super.getBlockStateAt(position);
            }
        };
    }

    private static RocketLandingFixture rocketLandingFixture(World world, PhysicsState initial) {
        var control = new ElytraFlightController.Decision(0, -20, false,
                ElytraFlightController.Phase.CRUISE);
        var lifetime = FireworkSchedule.lifetimeBounds(1);
        int ticks = lifetime.maximumPoweredTicks() + 2;
        var paths = new ArrayList<List<Vector3d>>();
        // Match runtime admission's immediate and next-tick observation witnesses,
        // but deliberately exclude the rejected path from the published geometry.
        for (int poweredTicks = lifetime.minimumPoweredTicks();
                poweredTicks <= lifetime.maximumPoweredTicks(); poweredTicks++) {
            for (int acknowledgementDelay = 0; acknowledgementDelay <= 1; acknowledgementDelay++) {
                var path = new ArrayList<Vector3d>();
                PhysicsState state = initial;
                path.add(state.position());
                for (int tick = 0; tick < ticks; tick++) {
                    var result = new GlidingPhysics(control.yaw(), control.pitch(),
                            tick >= acknowledgementDelay && tick < poweredTicks, PlayerInput.NONE,
                            MovementParameters.DEFAULT).step(world, state,
                                    PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
                    assertFalse(result.onGround(), "every powered lifetime must remain airborne");
                    assertFalse(result.collidedX() || result.collidedZ(),
                            "powered branches must avoid the rejection-only collision");
                    assertTrue(LocalAirRoute.visible(world, state.position(), result.position(),
                            PlayerPose.GLIDING));
                    var next = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                            result.fallDistance(), PlayerPose.GLIDING);
                    assertTrue(FlightHazards.safeTravel(world, state,
                            new PhysicsResult(result.position(), result.velocity(),
                                    result.collidedX(), result.collidedZ(), result.onGround(),
                                    result.fallDistance(), PlayerPose.GLIDING)),
                            "hot support must affect rejection, not the powered witnesses");
                    state = next;
                    path.add(state.position());
                }
                paths.add(path);
            }
        }
        var corridor = WaypointFlightTarget.executionWitness(paths,
                paths.get(0).get(paths.get(0).size() - 1), false);
        return new RocketLandingFixture(initial, control, repeatedSteps(initial, control, ticks), corridor);
    }

    private static List<PhysicsState> rejectedLanding(World world, RocketLandingFixture fixture) {
        var states = new ArrayList<PhysicsState>();
        PhysicsState state = fixture.initial();
        states.add(state);
        boolean departed = false;
        for (var step : fixture.steps()) {
            var control = step.control();
            var result = new GlidingPhysics(control.yaw(), control.pitch(), false, PlayerInput.NONE,
                    MovementParameters.DEFAULT).step(world, state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            departed |= !fixture.corridor().contains(state.position(), result.position());
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                    result.fallDistance(), PlayerPose.GLIDING);
            states.add(state);
            if (state.onGround()) break;
        }
        assertTrue(departed, "the rejected branch must genuinely leave the powered-only corridor");
        assertTrue(state.onGround(), "the fixture must reach support under the same fixed controls");
        return List.copyOf(states);
    }

    private static List<FlightControlAdmission.Step> repeatedSteps(PhysicsState state,
            ElytraFlightController.Decision control, int count) {
        var steps = new ArrayList<FlightControlAdmission.Step>(count);
        for (int tick = 0; tick < count; tick++) steps.add(new FlightControlAdmission.Step(control, state));
        return List.copyOf(steps);
    }

    private static List<PhysicsState> replay(PhysicsState initial,
            ElytraFlightController.Decision control, int ticks, int lifetime) {
        return replay(initial, control, ticks, 0, lifetime);
    }

    private static void assertFallThresholdCrossesOnLastTransition(List<PhysicsState> states) {
        assertTrue(states.stream().noneMatch(PhysicsState::onGround),
                "the fall margin, not a landing, must reject the replay");
        assertTrue(states.subList(0, states.size() - 1).stream()
                .allMatch(state -> state.fallDistance() < MovementParameters.DEFAULT.safeFallDistance()),
                "every pre-transition state must retain its fall margin");
        assertTrue(states.get(states.size() - 1).fallDistance() >= MovementParameters.DEFAULT.safeFallDistance(),
                "the final transition must really cross the fall-damage threshold");
    }

    private static List<PhysicsState> replay(PhysicsState initial,
            ElytraFlightController.Decision control, int ticks, int poweredFrom, int poweredUntil) {
        return replay(initial, repeatedSteps(initial, control, ticks), poweredFrom, poweredUntil);
    }

    private static List<PhysicsState> replay(PhysicsState initial,
            List<FlightControlAdmission.Step> steps, int poweredFrom, int poweredUntil) {
        var states = new ArrayList<PhysicsState>(steps.size() + 1);
        PhysicsState state = initial;
        states.add(state);
        for (int tick = 0; tick < steps.size(); tick++) {
            var control = steps.get(tick).control();
            boolean powered = tick >= poweredFrom && tick < poweredUntil;
            var result = new GlidingPhysics(control.yaw(), control.pitch(), powered,
                    PlayerInput.NONE, MovementParameters.DEFAULT).step(AIR, state,
                            PlayerPose.HALF_WIDTH, PlayerPose.GLIDING.height());
            state = new PhysicsState(result.position(), result.velocity(), result.onGround(),
                    result.fallDistance(), PlayerPose.GLIDING);
            states.add(state);
        }
        return List.copyOf(states);
    }

    private static PhysicsState state(double x) {
        return new PhysicsState(new Vector3d(x, 40, 8), new Vector3d(1, 0, 0),
                false, 0, PlayerPose.GLIDING);
    }

    private static World coast(double exposedFromX) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d position) {
                if (position.y < 32) return position.x >= exposedFromX ? LAVA : STONE;
                return super.getBlockStateAt(position);
            }
        };
    }
}
