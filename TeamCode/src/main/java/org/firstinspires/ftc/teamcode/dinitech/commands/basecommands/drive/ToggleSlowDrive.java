package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * An instant command that toggles the drive mode between normal speed and slow speed.
 * <p>
 * When executed, this command checks the current state of the {@link DriveSubsystem}.
 * <ul>
 *     <li>If the drive is in slow mode, it sets the default command back to {@link RobotCentricDrive} for
 *     normal-speed operation.</li>
 *     <li>If the drive is in normal mode, it sets the default command to {@link SlowDrive}
 *     for slow-speed, precise maneuvering, and triggers a rumble for haptic feedback.</li>
 * </ul>
 * The command finishes immediately after setting the new default command.
 */
public class ToggleSlowDrive extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final GamepadSubsystem gamepadSubsystem;

    /**
     * Creates a new ToggleSlowDrive command.
     *
     * @param driveSubsystem   The drive subsystem whose default command will be toggled.
     * @param visionSubsystem  The vision subsystem required by the locked drive mode.
     * @param gamepadSubsystem The gamepad subsystem for providing haptic feedback.
     */
    public ToggleSlowDrive(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
    }

    /**
     * Executes the toggle logic. This is run once when the command is scheduled.
     */
    @Override
    public void initialize() {
        // Cancel the current default command to allow a new one to be set.
        if (driveSubsystem.getDefaultCommand() != null) {
            driveSubsystem.getDefaultCommand().cancel();
        }

        if (driveSubsystem.getDriveUsage() == DriveSubsystem.DriveUsage.SLOW) {
            // If currently in slow mode, switch back to normal drive
            driveSubsystem.setDefaultCommand(new FieldCentricDrive(driveSubsystem, visionSubsystem, gamepadSubsystem));
        } else {
            // If in normal mode, switch to slow drive and provide feedback
            driveSubsystem.setDefaultCommand(new SlowDrive(driveSubsystem, gamepadSubsystem));
        }
    }

    /**
     * This command has no duration and is finished immediately after execution.
     * @return Always returns true.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
