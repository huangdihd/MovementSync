package xin.bbtt.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import xin.bbtt.MovementSync;
import xin.bbtt.movements.WalkMovement;
import xin.bbtt.physics.PlayerInput;

final class WalkCommandExecutorTest {
    private MovementSync movementSync;

    @BeforeEach
    void setUp() {
        movementSync = new MovementSync();
    }

    @AfterEach
    void tearDown() {
        MovementSync.INSTANCE = null;
    }

    @Test
    void defaultsToOneSecondAndWalking() {
        WalkMovement movement = WalkCommandExecutor.parseMovement(new String[]{"front"});
        assertEquals(1_000L, movement.getTime());
        movement.init();
        assertEquals(new PlayerInput(true, false, false, false, false, false, false), movementSync.getInput());
    }

    @Test
    void acceptsExplicitDurationAndSprint() {
        WalkMovement movement = WalkCommandExecutor.parseMovement(new String[]{"right", "250", "SPRINT"});
        assertEquals(250L, movement.getTime());
        movement.init();
        assertEquals(new PlayerInput(false, false, false, true, false, false, true), movementSync.getInput());
    }

    @Test
    void compassDirectionsSelectForwardInputAndTheirYaw() {
        String[] directions = {"north", "south", "west", "east"};
        float[] yaws = {180f, 0f, 90f, -90f};
        for (int index = 0; index < directions.length; index++) {
            WalkMovement movement = WalkCommandExecutor.parseMovement(new String[]{directions[index]});
            movement.init();
            assertEquals(0.0, Math.IEEEremainder(movementSync.yaw.get() - yaws[index], 360.0), 1.0e-6);
            assertEquals(new PlayerInput(true, false, false, false, false, false, false), movementSync.getInput());
            movement.onStop();
        }
    }

    @Test
    void rejectsInvalidArgumentsWithoutChangingInput() {
        List<String[]> invalid = List.of(
            new String[]{}, new String[]{"up"}, new String[]{"down"}, new String[]{"unknown"},
            new String[]{"front", "-1"}, new String[]{"front", "bad"},
            new String[]{"front", "9223372036854775808"},
            new String[]{"front", "10", "false"}, new String[]{"front", "10", "sprint", "extra"});
        for (String[] arguments : invalid) {
            assertThrows(IllegalArgumentException.class, () -> WalkCommandExecutor.parseMovement(arguments));
            assertEquals(PlayerInput.NONE, movementSync.getInput());
        }
    }

    @Test
    void completionAndParsingUseLocaleIndependentDirectionNames() {
        Locale previous = Locale.getDefault();
        try {
            Locale.setDefault(Locale.forLanguageTag("tr-TR"));
            WalkMovement movement = WalkCommandExecutor.parseMovement(new String[]{"right"});
            movement.init();
            assertEquals(new PlayerInput(false, false, false, true, false, false, false), movementSync.getInput());
            WalkCommandExecutor executor = new WalkCommandExecutor();
            assertEquals(List.of("RIGHT"), executor.onTabComplete(null, "walk", new String[]{"ri"}));
            assertEquals(List.of("sprint"), executor.onTabComplete(null, "walk", new String[]{"front", "250", "sp"}));
            assertEquals(List.of(), executor.onTabComplete(null, "walk", new String[]{"up"}));
        } finally {
            Locale.setDefault(previous);
        }
    }
}
