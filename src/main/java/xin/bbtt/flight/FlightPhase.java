package xin.bbtt.flight;

/** Extensible manoeuvre stage; plugins may supply their own enum. */
public interface FlightPhase {
    String name();
    enum Standard implements FlightPhase { CLIMB, CRUISE, DESCENT, AVOIDANCE, LANDING, FORCED_LANDING, LANDED }
}
