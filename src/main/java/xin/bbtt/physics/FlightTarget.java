package xin.bbtt.physics;

import org.joml.Vector3d;

/** A snapshot of flight intent; implementations return defensive vectors. */
public interface FlightTarget {
    Vector3d position();
    Vector3d velocity();
    boolean landed();
    boolean lost();
    default Vector3d aim(PhysicsState observer) { return position(); }
}
