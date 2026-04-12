package xin.bbtt.tasks;

import org.geysermc.mcprotocollib.protocol.packet.ingame.serverbound.player.ServerboundMovePlayerPosRotPacket;
import org.joml.Vector3d;
import xin.bbtt.MovementSync;
import xin.bbtt.mcbot.Bot;
import xin.bbtt.mcbot.Server;
import xin.bbtt.world.Direction;

import static xin.bbtt.MovementSync.gravitationalAcceleration;
import static xin.bbtt.MovementSync.terminalVelocity;

public class updateMotionTask implements Runnable {

    Vector3d lastPos = new Vector3d();
    float lastPitch = 0;
    float lastYaw = 0;

    public void syncPositionToServer() {
        Bot.Instance.getSession().send(new ServerboundMovePlayerPosRotPacket(
                MovementSync.Instance.onGround.get(),
                false,
                MovementSync.Instance.position.get().x,
                MovementSync.Instance.position.get().y,
                MovementSync.Instance.position.get().z,
                MovementSync.Instance.yaw.get(),
                MovementSync.Instance.pitch.get()

        ));
        MovementSync.Instance.getLogger().debug("Synced position to server: ({}, {}, {}, {}), vertical velocity: {}b/t", MovementSync.Instance.onGround, MovementSync.Instance.position.get().x, MovementSync.Instance.position.get().y, MovementSync.Instance.position.get().z, MovementSync.Instance.velocity.get().y);
    }

    public static void checkOnGround() {
        Vector3d position = new Vector3d(MovementSync.Instance.position.get());
        MovementSync.Instance.onGround.set(MovementSync.Instance.getWorld().isOnGround(position));
    }

    @Override
    public void run() {
        if (!Bot.Instance.isRunning()) return;
        if (Bot.Instance.getServer() != Server.Xin) return;
        Vector3d velocity = MovementSync.Instance.velocity.get();
        Vector3d displacement = new Vector3d();

        checkOnGround();
        if (MovementSync.Instance.onGround.get() && velocity.y < 0) {
            velocity.y = 0;
        }

        Vector3d position = new Vector3d(MovementSync.Instance.position.get());

        if (velocity.y > terminalVelocity) {
            if (!MovementSync.Instance.onGround.get()) velocity.add(gravitationalAcceleration);
            velocity.y *= 0.9800000190734863D;
            displacement.add(velocity);
        } else if (velocity.y < 0) {
            velocity.y = terminalVelocity;
            displacement.add(MovementSync.Instance.velocity.get().add(velocity).div(2));
        }
        Vector3d lowest = new Vector3d(position);
        lowest.y = Math.ceil(position.y);

        if (!MovementSync.Instance.onGround.get()) {
            while (!MovementSync.Instance.getWorld().isOnGround(lowest))
                lowest.add(Direction.DOWN.getUnitVector());
        }

        position.add(displacement);

        if (position.y < lowest.y){
            position.y = lowest.y;
        }

        MovementSync.Instance.velocity.set(velocity);
        MovementSync.Instance.position.set(position);

        if (!(lastPos.equals(MovementSync.Instance.position.get()) && lastPitch == MovementSync.Instance.pitch.get() && lastYaw == MovementSync.Instance.yaw.get()) && Bot.Instance.getServer() == Server.Xin) {
            checkOnGround();
            syncPositionToServer();
            lastPos = MovementSync.Instance.position.get();
            lastPitch = MovementSync.Instance.pitch.get();
            lastYaw = MovementSync.Instance.yaw.get();
        }
    }
}
