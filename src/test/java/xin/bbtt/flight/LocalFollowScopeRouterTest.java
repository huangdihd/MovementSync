package xin.bbtt.flight;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class LocalFollowScopeRouterTest {
    @Test void followsLocallyByDefaultIndependentlyOfGoto() {
        var router = new ScopeRouter();
        assertTrue(router.isLocalFollowEnabled());
        assertEquals(ScopeRouter.Route.LOCAL, router.route(ScopeRouter.Scope.LOCAL_FOLLOW, false, false));
        router.setLocalGotoEnabled(false);
        assertEquals(ScopeRouter.Route.LOCAL, router.route(ScopeRouter.Scope.LOCAL_FOLLOW, false, false));
        router.setLocalFollowEnabled(false);
        assertEquals(ScopeRouter.Route.LEGACY, router.route(ScopeRouter.Scope.LOCAL_FOLLOW, false, false));
        router.setLocalGotoEnabled(true);
        assertEquals(ScopeRouter.Route.LEGACY, router.route(ScopeRouter.Scope.LOCAL_FOLLOW, false, false));
        assertEquals(ScopeRouter.Route.LOCAL, router.route(ScopeRouter.Scope.UNDERROOF_GOTO, false, false));
        assertEquals(ScopeRouter.Route.LOCAL, router.route(ScopeRouter.Scope.LOCAL_CLIMB, false, false));
        router.setLocalFollowEnabled(true);
        assertEquals(ScopeRouter.Route.LOCAL, router.route(ScopeRouter.Scope.LOCAL_FOLLOW, false, false));
    }

    @Test void excludesEndAndPluginAndPreservesShadowFallback() {
        var router = new ScopeRouter();
        for (boolean enabled : new boolean[] {true, false}) {
            router.setLocalFollowEnabled(enabled);
            router.setShadowEnabled(true);
            assertEquals(ScopeRouter.Route.LEGACY, router.route(ScopeRouter.Scope.LOCAL_FOLLOW, true, false));
            assertEquals(ScopeRouter.Route.LEGACY, router.route(ScopeRouter.Scope.LOCAL_FOLLOW, false, true));
            assertEquals(ScopeRouter.Route.LEGACY, router.route(ScopeRouter.Scope.LOCAL_FOLLOW, true, true));
            assertEquals(enabled ? ScopeRouter.Route.LOCAL : ScopeRouter.Route.SHADOW,
                    router.route(ScopeRouter.Scope.LOCAL_FOLLOW, false, false));
        }
    }
}
