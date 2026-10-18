package xin.bbtt.flight;

/** P0 routing seam only: no local controller is executable yet. */
public final class ScopeRouter {
    public enum Scope { LEGACY, LOCAL_CLIMB, UNDERROOF_GOTO, LOCAL_FOLLOW }
    public enum Route { LEGACY, SHADOW }

    private volatile boolean shadowEnabled;

    /** Defaults to false. Enabling permits observation, never control publication. */
    public void setShadowEnabled(boolean enabled) { shadowEnabled = enabled; }
    public boolean isShadowEnabled() { return shadowEnabled; }

    public Route route(Scope scope, boolean endVoid, boolean pluginMode) {
        return shadowEnabled && scope != null && scope != Scope.LEGACY && !endVoid && !pluginMode
                ? Route.SHADOW : Route.LEGACY;
    }
}
