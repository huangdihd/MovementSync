package xin.bbtt.flight;

/**
 * Additive candidate-ranking cost; smaller totals are preferred.
 * Invoked synchronously during selection after safety filtering, potentially many
 * times. Shared implementations must be thread-safe and must not mutate live state.
 */
@FunctionalInterface
public interface FlightCostTerm {
    /**
     * Scores a main prediction without executing its action.
     * @param context non-null planning snapshot
     * @param action non-null proposed controls
     * @param trajectory non-null main prediction, not its recovery tail
     * @return additive cost; a nonfinite total rejects the candidate
     */
    double cost(FlightContext context, FlightAction action, Trajectory trajectory);
}
