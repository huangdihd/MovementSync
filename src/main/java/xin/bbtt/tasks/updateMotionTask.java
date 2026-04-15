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
    }

    public static void checkOnGround() {
        Vector3d position = new Vector3d(MovementSync.Instance.position.get());
        MovementSync.Instance.onGround.set(MovementSync.Instance.getWorld().isOnGround(position));
    }

    private boolean isWallCollision(Vector3d pos) {
        return MovementSync.Instance.getWorld().isSolid(pos);
    }

    private boolean isPlayerBoxColliding(Vector3d pos) {
        double w = 0.299;

        double[] heights = {0.0, 0.9, 1.79};

        for (double h : heights) {
            Vector3d[] corners = {
                    new Vector3d(pos.x - w, pos.y + h, pos.z - w),
                    new Vector3d(pos.x + w, pos.y + h, pos.z - w),
                    new Vector3d(pos.x - w, pos.y + h, pos.z + w),
                    new Vector3d(pos.x + w, pos.y + h, pos.z + w)
            };
            for (Vector3d corner : corners) {
                if (isWallCollision(corner)) return true;
            }
        }
        return false;
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
            displacement.add(new Vector3d(velocity).add(velocity).div(2));
        }

        if (displacement.x != 0) {
            Vector3d testPosX = new Vector3d(position.x + displacement.x, position.y, position.z);
            if (isPlayerBoxColliding(testPosX)) {
                displacement.x = 0;
                velocity.x = 0;
            } else {
                position.x += displacement.x;
            }
        }

        if (displacement.z != 0) {
            Vector3d testPosZ = new Vector3d(position.x, position.y, position.z + displacement.z);
            if (isPlayerBoxColliding(testPosZ)) {
                displacement.z = 0;
                velocity.z = 0;
            } else {
                position.z += displacement.z;
            }
        }

        if (displacement.y > 0) {
            Vector3d testPosY = new Vector3d(position.x, position.y + displacement.y, position.z);
            if (isPlayerBoxColliding(testPosY)) {
                displacement.y = 0;
                velocity.y = 0;
            }
        }

        Vector3d lowest = new Vector3d(position);
        lowest.y = Math.ceil(position.y);

        if (!MovementSync.Instance.onGround.get()) {
            while (!MovementSync.Instance.getWorld().isOnGround(lowest) && lowest.y > -64) {
                lowest.add(Direction.DOWN.getUnitVector());
            }
        }

        position.y += displacement.y;

        if (position.y < lowest.y){
            position.y = lowest.y;
            velocity.y = 0;
        }

        MovementSync.Instance.velocity.set(velocity);
        MovementSync.Instance.position.set(position);

        if (!(lastPos.equals(position) && lastPitch == MovementSync.Instance.pitch.get() && lastYaw == MovementSync.Instance.yaw.get())) {
            checkOnGround();
            syncPositionToServer();
            lastPos.set(position);
            lastPitch = MovementSync.Instance.pitch.get();
            lastYaw = MovementSync.Instance.yaw.get();
        }
    }
}