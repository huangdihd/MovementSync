package xin.bbtt.flight;

import java.util.Map;
import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.physics.HazardRecovery;
import xin.bbtt.physics.MovementParameters;
import xin.bbtt.physics.PhysicsState;
import xin.bbtt.physics.PlayerPose;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

final class DangerousEntryGateTest {
    private static final BlockState STONE = block("stone", true);
    private static final BlockState LAVA = block("lava", false);
    private static BlockState block(String name, boolean solid) {
        return new BlockState("minecraft:" + name, solid ? 1 : 0, Map.of(),
                solid ? "block" : "empty", 0, solid, "");
    }
    private static PhysicsState flying(double x) {
        return new PhysicsState(new Vector3d(x, 40, 8), new Vector3d(.8, 0, 0),
                false, 0, PlayerPose.GLIDING);
    }
    private static World floor(BlockState floor) {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                return p.y < 32 ? floor : super.getBlockStateAt(p);
            }
        };
    }

    @Test void safeSupportIsNotExposedButHotSupportIs() {
        assertFalse(DangerousEntryGate.exposed(floor(STONE), flying(8)));
        assertTrue(DangerousEntryGate.exposed(floor(LAVA), flying(8)));
        assertTrue(DangerousEntryGate.exposed(floor(block("magma_block", true)), flying(8)));
        assertTrue(DangerousEntryGate.exposed(floor(block("campfire", true)), flying(8)));
    }

    @Test void unknownAndLoadedVoidAreExposed() {
        assertTrue(DangerousEntryGate.exposed(new World(), flying(8)));
        assertTrue(DangerousEntryGate.exposed(floor(block("air", false)), flying(8)));
    }

    @Test void oneSupportedCornerDoesNotHideLava() {
        World coast = coast();
        assertTrue(DangerousEntryGate.exposed(coast, flying(9)));
        assertFalse(DangerousEntryGate.exposed(coast, flying(10)));
    }

    private static World coast() {
        return new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y < 32) return p.x >= 9 ? STONE : LAVA;
                return super.getBlockStateAt(p);
            }
        };
    }

    @Test void ongoingExposureAndSafeEndDoNotSearch() {
        // Null movement/context deliberately fail if these fast paths start a proof.
        assertTrue(DangerousEntryGate.permit(floor(LAVA), flying(8), flying(9), null, null));
        assertTrue(DangerousEntryGate.permit(floor(STONE), flying(8), flying(9), null, null));
    }

    @Test void newEntryRequiresRecoverableEnd() {
        assertTrue(DangerousEntryGate.permit(coast(), flying(10), flying(8),
                MovementParameters.DEFAULT, HazardRecovery.Context.EMPTY));
        World voidEdge = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.x >= 9 && p.y < 32) return STONE;
                return super.getBlockStateAt(p);
            }
        };
        // HazardRecovery's NOT_EXPOSED result for void is not an entry certificate.
        assertFalse(DangerousEntryGate.permit(voidEdge, flying(10), flying(8),
                MovementParameters.DEFAULT, HazardRecovery.Context.EMPTY));
    }

    @Test void unreachableEntryFailsClosed() {
        World distantBank = new World() {
            @Override public boolean chunkLoaded(int x, int z) { return true; }
            @Override public BlockState getBlockStateAt(Vector3d p) {
                if (p.y < 32) return p.x >= 10000 ? STONE : LAVA;
                return super.getBlockStateAt(p);
            }
        };
        assertFalse(DangerousEntryGate.permit(distantBank, flying(10001), flying(8),
                MovementParameters.DEFAULT, HazardRecovery.Context.EMPTY));
    }
}
