package xin.bbtt.physics;

import xin.bbtt.world.World;

/** Simulates one tick without mutating the state or world, or sending packets. */
public interface PhysicsStepper {
    PhysicsResult step(World world, PhysicsState state, double halfWidth, double height);
}
