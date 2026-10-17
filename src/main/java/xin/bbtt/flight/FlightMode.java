package xin.bbtt.flight;

import java.util.List;
import xin.bbtt.MovementSync;
import xin.bbtt.movement.Movement;

/**
 * Prediction hooks must be side-effect free. Execution is called under the navigation guard.
 * Every tick may select a different owner; execute must establish all controls it needs.
 * No activation callbacks are used: prediction must not activate a mode, and switching
 * must not leave persistent controls requiring a later deactivation. Stateful instances
 * should be scoped per navigation; globally registered instances must be thread-safe.
 * Server anti-cheat compatibility is the mode author's responsibility.
 */
public interface FlightMode {
    /** Stable diagnostic name; instances may maintain their own navigation state. */
    default String name() { return getClass().getSimpleName(); }
    /** Whether this mode can supply airborne controls for this tick. */
    default boolean isAvailable(FlightContext context) { return true; }
    List<? extends FlightAction> candidates(FlightContext context);
    Trajectory predict(FlightContext context, FlightAction action);
    /** Recovery starts at main.finalState(); only the owning mode supplies recovery. */
    default List<Trajectory> recover(FlightContext context, FlightAction action, Trajectory main) { return List.of(); }
    void execute(FlightContext context, FlightAction action, MovementSync sync);
    default FlightPhase phase(FlightContext context, FlightAction action) { return context.phase(); }
    /**
     * Creates a movement that hands off an airborne state to this mode, or {@code null}
     * when takeoff is unsupported/unavailable. Null means try another mode, not execute
     * a vanilla takeoff on this mode's behalf. Factories must not mutate live state.
     * The caller owns init/tick/stop, navigation guards and safety validation before
     * dispatch. TravelMovement does not invoke this hook yet (second migration segment).
     */
    default Movement createTakeoff(FlightContext context) { return null; }
    /**
     * Creates a movement that reaches a supported landing and finishes, or {@code null}
     * to decline. The caller tries another mode or retains safe airborne control; null
     * never authorizes disabling flight. Same dispatch/safety contract as takeoff.
     * TravelMovement integration is deferred to the second migration segment.
     */
    default Movement createLanding(FlightContext context) { return null; }
}
