package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * An instant command that toggles the drive mode between normal speed and slow speed.
 * <p>
 * When executed, this command checks the current state of the {@link DrivePedroSubsystem}.
 * <ul>
 *     <li>If the drive is in slow mode, it sets the default command back to {@link RobotCentricDrive} for
 *     normal-speed operation.</li>
 *     <li>If the drive is in normal mode, it sets the default command to {@link SlowDrive}
 *     for slow-speed, precise maneuvering, and triggers a rumble for haptic feedback.</li>
 * </ul>
 * The command finishes immediately after setting the new default command.
 */
public class ToggleSlowDrive extends CommandBase {
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final GamepadSubsystem gamepadSubsystem;

    /**
     * Creates a new ToggleSlowDrive command.
     *
     * @param drivePedroSubsystem   The drive subsystem whose default command will be toggled.
     * @param visionSubsystem  The vision subsystem required by the locked drive mode.
     * @param gamepadSubsystem The gamepad subsystem for providing haptic feedback.
     */
    public ToggleSlowDrive(DrivePedroSubsystem drivePedroSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
    }

    /**
     * Executes the toggle logic. This is run once when the command is scheduled.
     */
    @Override
    public void initialize() {
        if (drivePedroSubsystem.getDriveUsage() == DrivePedroSubsystem.DriveUsage.SLOW) {
            // If currently in slow mode, switch back to normal drive
            drivePedroSubsystem.setDefaultCommand(new FieldCentricDrive(drivePedroSubsystem,gamepadSubsystem));
        } else {
            // If in normal mode, switch to slow drive and provide feedback
            drivePedroSubsystem.setDefaultCommand(new SlowDrive(drivePedroSubsystem, gamepadSubsystem));
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
