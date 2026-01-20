package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * A hybrid drive command that provides vision-assisted "locking" onto AprilTags.
 * <p>
 * This command operates in one of two modes:
 * <ul>
 *     <li><b>Normal Mode:</b> When no AprilTags are detected, it functions as a standard
 *     robot-centric tele-operated drive.</li>
 *     <li><b>Locked Mode:</b> When an AprilTag is detected, it automatically adjusts the robot's
 *     rotation to keep the tag centered. This "locks" the robot's orientation towards the
 *     target. The driver can still strafe and move forward/backward, and can override
 *     the rotation with the right joystick.</li>
 * </ul>
 * The auto-aiming rotation power is calculated based on the bearing to the AprilTag,
 * passed through a custom power function to create a smooth response curve.
 */
public class AimLockedDrive extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final GamepadEx driver;

    /**
     * Creates a new TeleDriveLocked command.
     *
     * @param driveSubsystem   The drive subsystem to control.
     * @param visionSubsystem  The vision subsystem for AprilTag detection and bearing.
     * @param gamepadSubsystem The gamepad subsystem for driver inputs.
     */
    public AimLockedDrive(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
                          GamepadSubsystem gamepadSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.driver = gamepadSubsystem.getDriverEx();

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.setDriveUsage(DriveSubsystem.DriveUsage.AIM_LOCKED);
    }

    /**
     * Switches between locked and normal drive modes based on AprilTag visibility.
     */
    @Override
    public void execute() {

        double rightX = driver.getRightX();

        if (visionSubsystem.getHasCurrentAprilTagDetections()) {
            // Execute the drive command with the combined rotation power
            double autoAimPower = visionSubsystem.getAutoAimPower();
            driveSubsystem.getTelemetry().addData("AT Locked :closer to 0", "%.4f", autoAimPower);
            driveSubsystem.teleDriveHybrid(driver.getLeftX(), driver.getLeftY(), autoAimPower * (1 - Math.abs(rightX)) + rightX, driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), driveSubsystem.getDriveReference() == DriveSubsystem.DriveReference.FC);
        } else {
            // Fallback to standard tele-op drive if no tags are visible
            driveSubsystem.teleDriveHybrid(driver.getLeftX(), driver.getLeftY(), rightX, driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), driveSubsystem.getDriveReference() == DriveSubsystem.DriveReference.FC);
        }
    }
}
