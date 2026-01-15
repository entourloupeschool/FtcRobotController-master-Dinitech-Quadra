package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.Rumble;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * An instant command that toggles the drive mode between normal tele-op and vision-assisted "locked" driving.
 * <p>
 * When executed, this command checks the AprilTag lock state of the {@link DriveSubsystem}.
 * <ul>
 *     <li>If the drive is in vision-locked mode, it restores the default command to {@link TeleDrive}
 *     for normal, robot-centric control.</li>
 *     <li>If the drive is in normal mode, it sets the default command to {@link AprilTagLockedTeleDrive},
 *     enabling the vision-assisted auto-aim behavior, and triggers a rumble for haptic feedback.</li>
 * </ul>
 * The command finishes immediately after setting the new default command.
 */
public class ToggleVisionDrive extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final GamepadSubsystem gamepadSubsystem;

    /**
     * Creates a new ToggleVisionDrive command.
     *
     * @param driveSubsystem   The drive subsystem to control.
     * @param visionSubsystem  The vision subsystem required by the locked drive mode.
     * @param gamepadSubsystem The gamepad subsystem for providing feedback.
     */
    public ToggleVisionDrive(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
            GamepadSubsystem gamepadSubsystem) {
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

        if (driveSubsystem.getUsageState() == DriveSubsystem.DriveUsageState.VISION) {
            // If vision is locked, switch back to normal drive.
            driveSubsystem.setDefaultCommand(new TeleDrive(driveSubsystem, gamepadSubsystem));
        } else {
            // If not locked, switch to vision-locked drive and provide feedback.
            new Rumble(gamepadSubsystem, 3, 3).schedule();
            driveSubsystem.setDefaultCommand(new AprilTagLockedTeleDrive(driveSubsystem, visionSubsystem, gamepadSubsystem));
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
