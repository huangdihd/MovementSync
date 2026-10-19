package xin.bbtt.physics;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.flight.DangerousEntryGate;
import xin.bbtt.world.World;
import static org.junit.jupiter.api.Assertions.*;

final class KnownLandingApproachTest {
    private static final BlockState STONE = new BlockState("minecraft:stone", 1, Map.of(), "block", 0, true, "");
    private static final BlockState LAVA = new BlockState("minecraft:lava", 0, Map.of(), "empty", 0, false, "");
    private static World coast() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y >= 120 || p.x < 0 && p.y < 40) return STONE;
                return p.y < 31 ? LAVA : super.getBlockStateAt(p);
            }
        };
    }
    private static HazardRecovery.Context resources() {
        return new HazardRecovery.Context(new int[]{1, 1, 1, 1, 1, 1, 1, 1}, 0, false, null);
    }
    @Test void elevatedApproachAvoidsGroundChordThroughShoreCliff() {
        World world = coast();
        var state = new PhysicsState(new Vector3d(70, 38, .5), new Vector3d(-.8, 0, 0), false);
        var site = new Vector3d(-8.5, 40, .5);
        assertFalse(KnownLandingReserve.corridor(world, state.position(), site));
        var result = KnownLandingReserve.assess(world, state, MovementParameters.DEFAULT,
                resources(), List.of(site), 10, 1);
        assertEquals(KnownLandingReserve.Status.VALID, result.status());
        assertEquals(site, result.returnSite());
    }
    @Test void duplicateObservationsDoNotEraseSupportedAnchor() {
        var result = KnownLandingReserve.assess(coast(),
                new PhysicsState(new Vector3d(70, 50, .5), new Vector3d(-.8, 0, 0), false),
                MovementParameters.DEFAULT, resources(), Collections.nCopies(200, new Vector3d(-8.5, 40, .5)), 10, 1);
        assertEquals(KnownLandingReserve.Status.VALID, result.status());
    }
    @Test void blockedNearestSiteDoesNotHideViableSupportedAlternative() {
        World world = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y < 40 || p.x < -3 && p.x > -14 && p.y >= 42 && p.y < 46) return STONE;
                return super.getBlockStateAt(p);
            }
        };
        var farther = new Vector3d(25, 40, .5);
        var result = KnownLandingReserve.assess(world,
                new PhysicsState(new Vector3d(0, 50, .5), new Vector3d(.8, 0, 0), false),
                MovementParameters.DEFAULT, resources(), List.of(new Vector3d(-8, 40, .5), farther), 10, 1);
        assertEquals(KnownLandingReserve.Status.VALID, result.status());
        assertEquals(farther, result.returnSite());
    }
    @Test void lavaTrajectoryCannotQualifyAsApproach() {
        var state = new PhysicsState(new Vector3d(70, 29, .5), new Vector3d(-.8, 0, 0), false);
        assertFalse(GlideRange.scheduledReach(coast(), state, MovementParameters.DEFAULT,
                new Vector3d(-8.5, 44, .5), new int[]{1, 1}, 2, 0, 0, 10));
    }
    @Test void completeLoadedProbeGridIsNotTruncatedAtSixtyFive() {
        var state = new PhysicsState(new Vector3d(-100, 50, .5), new Vector3d(), false);
        var sites = DangerousEntryGate.knownSites(coast(), state, resources());
        assertEquals(81, sites.size());
        assertEquals(sites.size(), sites.stream().distinct().count());
    }
}
