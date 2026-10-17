package xin.bbtt.physics;

import org.joml.Vector3d;

/** The original fixed destination, without prediction or arithmetic on its coordinates. */
public final class FixedFlightTarget implements FlightTarget {
    private final Vector3d position;
    public FixedFlightTarget(Vector3d position) {
        if (position == null || !position.isFinite()) throw new IllegalArgumentException("Target must be finite");
        this.position = new Vector3d(position);
    }
    @Override public Vector3d position() { return new Vector3d(position); }
    @Override public Vector3d velocity() { return new Vector3d(); }
    @Override public boolean landed() { return true; }
    @Override public boolean lost() { return false; }
}
