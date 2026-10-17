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
    /**
     * Whether this mode still controls the airborne state, queried before a decision.
     * Pure observation: do not mutate live state or acquire navigation locks while
     * holding the sync monitor. The default supports non-gliding flight until grounded.
     */
    default boolean isAirborne(FlightContext context, MovementSync sync) { return !context.state().onGround(); }
    /** Returns non-null candidate controls for this decision; an empty list declines participation. Pure query. */
    List<? extends FlightAction> candidates(FlightContext context);
    /**
     * Previews one candidate from context.state(), using the same physics as execution.
     * Called during selection, without live side effects. The result must be non-null;
     * omit unpreviewable actions from candidates instead. Start at the supplied state.
     */
    Trajectory predict(FlightContext context, FlightAction action);
    /** Recovery starts at main.finalState(); only the owning mode supplies recovery. */
    default List<Trajectory> recover(FlightContext context, FlightAction action, Trajectory main) { return List.of(); }
    /**
     * Applies the selected controls on the navigation thread under the navigation guard.
     * May mutate live controls; acquire the sync monitor only after the navigation lock.
     * Must not wait for physics ticks or leave controls requiring deactivation callbacks.
     */
    void execute(FlightContext context, FlightAction action, MovementSync sync);
    /** Returns a non-null diagnostic phase for a previewed action; pure query during selection. */
    default FlightPhase phase(FlightContext context, FlightAction action) { return context.phase(); }
    /**
     * Creates a movement that hands off an airborne state to this mode, or {@code null}
     * when takeoff is unsupported/unavailable. Null means try another mode, not execute
     * a vanilla takeoff on this mode's behalf. Factories must not mutate live state.
     * The caller owns init/tick/stop, navigation guards and safety validation before
     * dispatch. A non-null safety preview is required before this factory is invoked.
     */
    default Movement createTakeoff(FlightContext context) { return null; }
    /**
     * Creates a movement that reaches a supported landing and finishes, or {@code null}
     * to decline. The caller tries another mode or retains safe airborne control; null
     * never authorizes disabling flight. Same dispatch/safety contract as takeoff.
     */
    default Movement createLanding(FlightContext context) { return null; }
    /** Side-effect-free takeoff prediction; null declines. Supply the transition action on the trajectory. */
    default Trajectory previewTakeoff(FlightContext context) { return null; }
    /** Side-effect-free landing prediction; null declines. Supply the transition action on the trajectory. */
    default Trajectory previewLanding(FlightContext context) { return null; }
    /** Recovery must begin at the takeoff preview endpoint and belong to this mode. */
    default List<Trajectory> recoverTakeoff(FlightContext context, Trajectory preview) {
        return recover(context, preview.action(), preview);
    }
    /** Recovery must begin at the landing preview endpoint and belong to this mode. */
    default List<Trajectory> recoverLanding(FlightContext context, Trajectory preview) {
        return recover(context, preview.action(), preview);
    }
}
