package xin.bbtt.physics;

/** Internal policy seam. Cruise stays cost-only; emergency uses damage/approach/cost. */
enum FlightRanking {
    COST_ONLY, DAMAGE_FIRST;

    boolean better(FlightSafety.EmergencyCost cost, FlightSafety.EmergencyCost best) {
        return this == DAMAGE_FIRST ? cost.compareTo(best) < 0 : better(cost.score(), best.score());
    }

    boolean better(double score, double bestScore) {
        return !(score >= bestScore);
    }
}
