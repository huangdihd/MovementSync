package xin.bbtt.physics;

/** The stage of a flight manoeuvre; built-in phases retain their existing names. */
public interface FlightPhase extends xin.bbtt.flight.FlightPhase {
    String name();
}
