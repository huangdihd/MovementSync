package xin.bbtt.flight;

/** Additive ranking cost; nonfinite values reject a candidate. */
@FunctionalInterface
public interface FlightCostTerm {
    double cost(FlightContext context, FlightAction action, Trajectory trajectory);
}
