package xin.bbtt.physics;

import java.util.List;

/** Executable prefix only: diagnostic lookahead never extends this authorization. */
public record TickControlBatch(long generation, long epoch, long admittedAt, List<Command> commands) {
    public record Command(PhysicsState expected, float yaw, float pitch, boolean firework,
                          long validFrom, long validThrough) {}
    public TickControlBatch { commands = List.copyOf(commands); }

    public static TickControlBatch admitted(FlightControlAdmission.Envelope envelope, long tick) {
        var commands = new java.util.ArrayList<Command>();
        for (int i = 0; i < envelope.steps().size(); i++) {
            var step = envelope.steps().get(i);
            commands.add(new Command(step.expected(), step.control().yaw(), step.control().pitch(),
                    i == 0 && step.control().firework(), tick + i, tick + i));
        }
        return new TickControlBatch(envelope.source().generation(), envelope.source().epoch(), tick, commands);
    }

    public boolean authorizes(int index, long tick) {
        return index >= 0 && index < commands.size()
                && tick >= commands.get(index).validFrom() && tick <= commands.get(index).validThrough();
    }
}
