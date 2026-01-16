package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechMecanumDrive;

import java.util.Set;

/**
 * A command that wraps a Road Runner {@link Action} to make it compatible with the FTCLib command-based framework.
 * <p>
 * This class serves as a bridge between Road Runner's action-based autonomous system and the
 * command-based structure. It takes a Road Runner {@code Action} (which typically represents a
 * trajectory or a turn) and executes it within the {@code execute()} method of a standard
 * FTCLib command. It also handles sending telemetry data to the FTC Dashboard for field visualization.
 * <p>
 * Optionally, a {@link VisionSubsystem} can be provided to enable AprilTag-enhanced localization
 * during trajectory following. When provided, the command temporarily swaps the drive's localizer
 * to an {@link AprilTagLocalizer} for improved accuracy.
 */
public class FollowTrajectory extends CommandBase {
    private final Action action;
    private final DinitechMecanumDrive drive;
    private final VisionSubsystem visionSubsystem;
    
    private Localizer originalLocalizer;
    private AprilTagLocalizer aprilTagLocalizer;

    /**
     * Creates a new FollowTrajectory command without vision-enhanced localization.
     *
     * @param action         The Road Runner {@link Action} to be executed.
     * @param driveSubsystem The drive subsystem (required for trajectory following).
     */
    public FollowTrajectory(Action action, DriveSubsystem driveSubsystem) {
        this(action, driveSubsystem, null);
    }

    /**
     * Creates a new FollowTrajectory command with optional vision-enhanced localization.
     *
     * @param action          The Road Runner {@link Action} to be executed.
     * @param driveSubsystem  The drive subsystem (required for trajectory following).
     * @param visionSubsystem The vision subsystem for AprilTag localization (can be null).
     */
    public FollowTrajectory(Action action, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.action = action;
        this.drive = driveSubsystem.getDrive();
        this.visionSubsystem = visionSubsystem;

        addRequirements(driveSubsystem);
    }

    /**
     * Initializes the command. If a VisionSubsystem is provided, swaps the localizer
     * to an AprilTagLocalizer for enhanced accuracy during trajectory following.
     */
    @Override
    public void initialize() {
        if (visionSubsystem != null) {
            // Store the original localizer to restore later
            originalLocalizer = drive.localizer;
            
            // Create and set the AprilTagLocalizer with the original as base
            aprilTagLocalizer = new AprilTagLocalizer(originalLocalizer, visionSubsystem);
            drive.localizer = aprilTagLocalizer;
        }
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
     * Called when the command ends. Restores the original localizer if it was swapped.
     *
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        // Restore the original localizer if we swapped it
        if (originalLocalizer != null) {
            drive.localizer = originalLocalizer;
            originalLocalizer = null;
            aprilTagLocalizer = null;
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
