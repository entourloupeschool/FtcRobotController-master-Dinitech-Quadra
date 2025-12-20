package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.gamepad.Rumble;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;

/**
 * An instant command that toggles the drive mode between normal speed and slow speed.
 * <p>
 * When executed, this command checks the current state of the {@link DriveSubsystem}.
 * <ul>
 *     <li>If the drive is in slow mode, it sets the default command back to {@link TeleDrive} for
 *     normal-speed operation.</li>
 *     <li>If the drive is in normal mode, it sets the default command to {@link TeleSlowDrive}
 *     for slow-speed, precise maneuvering, and triggers a rumble for haptic feedback.</li>
 * </ul>
 * The command finishes immediately after setting the new default command.
 */
public class ToggleSlowDrive extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final GamepadSubsystem gamepadSubsystem;

    /**
     * Creates a new ToggleSlowDrive command.
     *
     * @param driveSubsystem   The drive subsystem whose default command will be toggled.
     * @param gamepadSubsystem The gamepad subsystem for providing haptic feedback.
     */
    public ToggleSlowDrive(DriveSubsystem driveSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
    }

    /**
     * Executes the toggle logic. This is run once when the command is scheduled.
     */
    @Override
    public void initialize() {
        // Cancel the current default command to allow a new one to be set
        driveSubsystem.getDefaultCommand().cancel();

        if (driveSubsystem.isSlowDrive()) {
            // If currently in slow mode, switch back to normal drive
            driveSubsystem.setDefaultCommand(new TeleDrive(driveSubsystem, gamepadSubsystem));
            driveSubsystem.setIsSlowDrive(false);
        } else {
            // If in normal mode, switch to slow drive and provide feedback
            new Rumble(gamepadSubsystem, 3, 3).schedule(); // Rumble to indicate mode change
            driveSubsystem.setDefaultCommand(new TeleSlowDrive(driveSubsystem, gamepadSubsystem));
            driveSubsystem.setIsSlowDrive(true);
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
