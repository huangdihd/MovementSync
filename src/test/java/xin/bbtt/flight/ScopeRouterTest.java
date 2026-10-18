package xin.bbtt.flight;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class ScopeRouterTest {
    @Test void defaultsToLocalForGotoAndDeploymentClimb() {
        var router = new ScopeRouter();
        assertFalse(router.isShadowEnabled());
        for (var scope : ScopeRouter.Scope.values()) {
            assertEquals(scope == ScopeRouter.Scope.UNDERROOF_GOTO || scope == ScopeRouter.Scope.LOCAL_CLIMB
                    ? ScopeRouter.Route.LOCAL
                    : ScopeRouter.Route.LEGACY, router.route(scope, false, false));
        }
    }

    @Test void optInOnlySelectsShadowWithinExplicitLocalScopes() {
        var router = new ScopeRouter();
        router.setLocalGotoEnabled(false);
        router.setShadowEnabled(true);
        for (var scope : ScopeRouter.Scope.values()) {
            assertEquals(scope == ScopeRouter.Scope.LEGACY ? ScopeRouter.Route.LEGACY
                    : ScopeRouter.Route.SHADOW, router.route(scope, false, false));
            assertEquals(ScopeRouter.Route.LEGACY, router.route(scope, true, false));
            assertEquals(ScopeRouter.Route.LEGACY, router.route(scope, false, true));
        }
        assertEquals(ScopeRouter.Route.LEGACY, router.route(null, false, false));
    }

    @Test void disablingRestoresLegacyWithoutRememberingPreviousScope() {
        var router = new ScopeRouter();
        router.setLocalGotoEnabled(false);
        router.setShadowEnabled(true);
        assertEquals(ScopeRouter.Route.SHADOW, router.route(ScopeRouter.Scope.LOCAL_FOLLOW, false, false));
        router.setShadowEnabled(false);
        assertEquals(ScopeRouter.Route.LEGACY, router.route(ScopeRouter.Scope.LOCAL_FOLLOW, false, false));
    }
}
