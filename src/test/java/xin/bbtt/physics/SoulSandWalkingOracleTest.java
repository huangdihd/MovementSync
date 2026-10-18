package xin.bbtt.physics;

import org.joml.Vector3d;
import org.junit.jupiter.api.Test;
import xin.bbtt.Block.BlockState;
import xin.bbtt.Block.BlockStateParser;
import xin.bbtt.world.World;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Independent axis-aligned vanilla oracle: no production collision or multiplier helpers.
 * Verified against Mojang's 1.21.11 client (SHA-1 ba2df812c2d12e0219c489c4cd9a5e1f0760f5bd):
 * Entity cgk.bi/bf/f = Yarn getVelocityMultiplier/getVelocityAffectingPos/getPosWithYOffset;
 * cgk.b(boolean, ftm) probes support, dvx.j picks the nearest supporting block;
 * SoulSandBlock ehr's shape is 16 x 14 x 16; Blocks dzs registers sand speed 0.4f,
 * whereas soul_soil uses the default speed. Yarn mapping names are published at
 * https://github.com/FabricMC/yarn/blob/1.21.11/mappings/net/minecraft/entity/Entity.mapping .
 */
final class SoulSandWalkingOracleTest {
    private static final double HALF = (double) 0.6f / 2;
    private static final double ACCELERATION = (double) 0.1f * 0.98 * 0.21600002 / (0.6 * 0.6 * 0.6);

    @Test
    void alternatingSandAndSoilMatchesEveryTickInBothDirections() {
        for (int direction : new int[]{1, -1}) {
            World world = new AlternatingWorld();
            WalkingPhysics physics = new WalkingPhysics(
                    new PlayerInput(true, false, false, false, false, false, false), direction * -90);
            double x = 30.5, y = 32.875, vx = 0, vy = -0.0784;
            boolean grounded = true;
            PhysicsState state = new PhysicsState(new Vector3d(x, y, -0.5), new Vector3d(vx, vy, 0), grounded);
            int boundarySupportTicks = 0;
            for (int tick = 0; tick < 240; tick++) {
                double previousX = x;
                double previousY = y;
                boolean wasGrounded = grounded;
                if (Math.abs(vx) < 0.003) vx = 0;
                if (Math.abs(vy) < 0.003) vy = 0;
                vx += direction * (grounded ? ACCELERATION : 0.02 * 0.98);
                // Vanilla resolves downward Y before horizontal travel.
                double floor = highestTop(x);
                y = Math.max(y + vy, floor);
                grounded = y > previousY + vy;
                if (grounded) vy = 0;
                x += vx;
                // Rising 1/8-block transitions fit the player's vanilla step height.
                double nextFloor = highestTop(x);
                if (nextFloor > y && (wasGrounded || grounded)) y = nextFloor;
                int feet = (int) Math.floor(x);
                double multiplier = 1;
                if (Math.floor(y) == 32 && (feet & 1) == 0) multiplier = (double) 0.4f;
                else {
                    int support = supportingColumn(x, y);
                    if (grounded && support == Integer.MIN_VALUE) support = supportingColumn(previousX, y);
                    if (!grounded || support == Integer.MIN_VALUE) support = feet;
                    if ((int) Math.floor(y - (double) 0.500001f) == 32 && (support & 1) == 0) {
                        multiplier = (double) 0.4f;
                    }
                    if (grounded && support != feet) boundarySupportTicks++;
                }
                vx *= multiplier * (wasGrounded ? 0.6 * 0.91 : 0.91);
                vy = (vy - 0.08) * (double) 0.98f;
                PhysicsResult actual = physics.step(world, state, HALF, 1.8);
                String context = "direction=" + direction + " tick=" + tick + " actualY=" + actual.position().y;
                assertEquals(x, actual.position().x, 1e-9, context + " x");
                assertEquals(y, actual.position().y, 1e-9, context + " y");
                assertEquals(vx, actual.velocity().x, 1e-9, context + " vx");
                assertEquals(vy, actual.velocity().y, 1e-9, context + " vy");
                assertEquals(grounded, actual.onGround(), context + " ground");
                state = new PhysicsState(actual.position(), actual.velocity(), actual.onGround());
            }
            assertTrue(boundarySupportTicks > 10, "must exercise off-center support repeatedly");
            assertTrue(Math.abs(x - 30.5) > 20, "no boundary sticking");
        }
    }

    @Test
    void reportedBoundaryHasSoilSupportNotCenterSandSlowdown() {
        // At x=30.7 the float-width player already overlaps the full-height soil at x=31.
        Vector3d position = new Vector3d(30.7, 33, -0.5);
        assertEquals(1, EnvironmentalPhysics.velocityMultiplier(new AlternatingWorld(), position,
                HALF, true, new Vector3d(0.1, 0, 0)));
    }

    private static double highestTop(double x) {
        double top = 32.875;
        for (int column = (int) Math.floor(x - HALF); column < Math.ceil(x + HALF); column++) {
            if ((column & 1) != 0) top = 33;
        }
        return top;
    }

    private static int supportingColumn(double x, double y) {
        int best = Integer.MIN_VALUE;
        double distance = Double.POSITIVE_INFINITY;
        for (int column = (int) Math.floor(x - HALF); column < Math.ceil(x + HALF); column++) {
            double top = (column & 1) == 0 ? 32.875 : 33;
            if (top <= y - 1e-6 || top > y) continue;
            double candidate = Math.abs(column + 0.5 - x);
            if (candidate <= distance) { best = column; distance = candidate; }
        }
        return best;
    }

    private static BlockState state(String name) {
        for (int id = 0; id < BlockStateParser.getBlockStateRegistrySize(); id++) {
            BlockState state = BlockStateParser.Instance.parseStateId(id);
            if (state.blockName().equals(name)) return state;
        }
        throw new AssertionError(name);
    }

    private static final class AlternatingWorld extends World {
        private final BlockState sand = state("soul_sand"), soil = state("soul_soil"), air = state("air");
        @Override public boolean chunkLoaded(int x, int z) { return true; }
        @Override public BlockState getBlockStateAt(Vector3d position) {
            return Math.floor(position.y) == 32 ? (((int) Math.floor(position.x) & 1) == 0 ? sand : soil) : air;
        }
    }
}
