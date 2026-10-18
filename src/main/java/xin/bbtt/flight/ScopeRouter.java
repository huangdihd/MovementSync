package xin.bbtt.flight;

/** Local goto control routing; other scopes retain their existing controller. */
public final class ScopeRouter {
    public enum Scope { LEGACY, LOCAL_CLIMB, UNDERROOF_GOTO, LOCAL_FOLLOW }
    public enum Route { LEGACY, SHADOW, LOCAL }

    private volatile boolean shadowEnabled;
    private volatile boolean localGotoEnabled = true;

    public void setLocalGotoEnabled(boolean enabled) { localGotoEnabled = enabled; }
    public boolean isLocalGotoEnabled() { return localGotoEnabled; }

    /** Defaults to false. Enabling permits observation, never control publication. */
    public void setShadowEnabled(boolean enabled) { shadowEnabled = enabled; }
    public boolean isShadowEnabled() { return shadowEnabled; }

    public Route route(Scope scope, boolean endVoid, boolean pluginMode) {
        if (localGotoEnabled && (scope == Scope.UNDERROOF_GOTO || scope == Scope.LOCAL_CLIMB)
                && !endVoid && !pluginMode)
            return Route.LOCAL;
        return shadowEnabled && scope != null && scope != Scope.LEGACY && !endVoid && !pluginMode
                ? Route.SHADOW : Route.LEGACY;
    }
}
