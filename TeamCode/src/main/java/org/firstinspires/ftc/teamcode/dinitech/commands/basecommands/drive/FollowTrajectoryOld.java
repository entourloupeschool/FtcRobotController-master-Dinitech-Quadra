package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.Set;

/**
 * A command that wraps a Road Runner {@link Action} to make it compatible with the FTCLib command-based framework.
 * <p>
 * This class serves as a bridge between Road Runner's action-based autonomous system and the
 * command-based structure. It takes a Road Runner {@code Action} (which typically represents a
 * trajectory or a turn) and executes it within the {@code execute()} method of a standard
 * FTCLib command. It also handles sending telemetry data to the FTC Dashboard for field visualization.
 */
public class FollowTrajectory extends CommandBase {
    private final Action action;

    /**
     * Creates a new FollowTrajectory command.
     *
     * @param action       The Road Runner {@link Action} to be executed.
     * @param requirements The set of {@link Subsystem}s required by this command.
     */
    public FollowTrajectory(Action action, Set<Subsystem> requirements) {
        this.action = action;
        addRequirements(requirements.toArray(new Subsystem[0]));
    }

    /**
     * Executes the wrapped Road Runner action and sends telemetry.
     * <p>
     * This method is called repeatedly by the command scheduler. In each iteration, it runs the
     * action and checks if it has completed. It also sends a {@link TelemetryPacket} to the
     * FTC Dashboard, which includes field overlay information for visualization.
     */
    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        
        // The run() method of an Action returns true if it is still running and false when it is finished.
        boolean running = action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        
        if (!running) {
            // End the command when the action is finished.
            end(false);
        }
    }

    /**
     * This command is considered finished when the wrapped action is finished.
     * FTCLib's isFinished() is implicitly handled by the execute loop ending the command.
     */
    @Override
    public boolean isFinished() {
        // This is managed by the call to end() within execute(), but returning false here
        // ensures the command continues scheduling until explicitly ended.
        return super.isFinished();
    }
}
