package xin.bbtt;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;
import org.geysermc.mcprotocollib.network.ClientSession;
import org.geysermc.mcprotocollib.protocol.data.game.item.ItemStack;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import xin.bbtt.inventory.ItemRegistry;
import xin.bbtt.mcbot.Bot;

/** Isolates the global client and records protocol requests without a server. */
public abstract class ElytraCommandTestSupport {
    protected final List<Object> packets = new ArrayList<>();
    protected MovementSync movementSync;
    protected ItemStack[] inventory;
    private MovementSync previousMovementSync;
    private Object previousSession;
    private Field sessionField;

    @BeforeEach
    void installClient() throws Exception {
        previousMovementSync = MovementSync.INSTANCE;
        sessionField = Bot.class.getDeclaredField("session");
        sessionField.setAccessible(true);
        previousSession = sessionField.get(Bot.INSTANCE);
        sessionField.set(Bot.INSTANCE, Proxy.newProxyInstance(
                ClientSession.class.getClassLoader(), new Class<?>[]{ClientSession.class},
                (proxy, method, arguments) -> {
                    if (method.getName().equals("isConnected")) return true;
                    if (method.getName().equals("send")) {
                        packets.add(arguments[0]);
                        return null;
                    }
                    if (method.getName().equals("toString")) return "RecordingClientSession";
                    if (method.getName().equals("hashCode")) return System.identityHashCode(proxy);
                    if (method.getName().equals("equals")) return proxy == arguments[0];
                    throw new UnsupportedOperationException(method.getName());
                }));
        movementSync = new MovementSync();
        movementSync.entityId = 42;
        inventory = new ItemStack[46];
        inventory[6] = item("elytra", 1);
        movementSync.inventoryManager.setContainerItems(0, inventory);
    }

    @AfterEach
    void restoreClient() throws Exception {
        try {
            if (movementSync != null && movementSync.movementService != null) {
                movementSync.movementService.shutdownNow();
            }
        } finally {
            MovementSync.INSTANCE = previousMovementSync;
            if (sessionField != null) sessionField.set(Bot.INSTANCE, previousSession);
        }
    }

    protected static ItemStack item(String name, int amount) {
        return new ItemStack(ItemRegistry.Instance.getItem(name).getId(), amount);
    }
}
