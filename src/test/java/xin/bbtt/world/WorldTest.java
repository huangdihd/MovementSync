package xin.bbtt.world;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class WorldTest {

    @Test
    void defaultValueIsVanillaOverworld() {
        // Without dimension registry data, defaults to vanilla overworld.
        assertEquals(-64, World.getMinWorldY());
        assertEquals(319, World.getMaxWorldY());
    }

    @Test
    void withinFallbackBoundsAtMin() {
        assertTrue(World.isWithinWorldBounds(World.getMinWorldY()));
    }

    @Test
    void withinFallbackBoundsAtMax() {
        assertTrue(World.isWithinWorldBounds(World.getMaxWorldY()));
    }

    @Test
    void withinBoundsAtZero() {
        assertTrue(World.isWithinWorldBounds(0));
    }

    @Test
    void belowMinIsOutOfBounds() {
        assertFalse(World.isWithinWorldBounds(World.getMinWorldY() - 1));
    }

    @Test
    void aboveMaxIsOutOfBounds() {
        assertFalse(World.isWithinWorldBounds(World.getMaxWorldY() + 1));
    }
}
