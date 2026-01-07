package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLAMP_BEARING;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.NUMBER_CUSTOM_POWER_FUNC_DRIVE_LOCKED;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CUSTOM_POWER_LOCKED;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SCALER_OFFSET_AT_TO_X_BASKET;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.pickCustomPowerFunc;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdateAprilTagsDetections;
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
public class TeleDriveLocked extends CommandBase {
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
    public TeleDriveLocked(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
            GamepadSubsystem gamepadSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.driver = gamepadSubsystem.getDriverEx();

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        if (visionSubsystem.getDefaultCommand() != null) {
            visionSubsystem.getDefaultCommand().cancel();
        }
        visionSubsystem.setDefaultCommand(new ContinuousUpdateAprilTagsDetections(visionSubsystem));
//        visionSubsystem.setCameraOn(true);
    }

    /**
     * Switches between locked and normal drive modes based on AprilTag visibility.
     */
    @Override
    public void execute() {

        if (visionSubsystem.getHasCurrentAprilTagDetections()) {
            withCurrentDetection();
        } else {
            // Fallback to standard tele-op drive if no tags are visible
            driveSubsystem.teleDrive(driver.getLeftX(), driver.getLeftY(), driver.getRightX(),
                    driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        }
    }

    @Override
    public void end(boolean interrupted) {
//        visionSubsystem.setCameraOn(false);
        if (visionSubsystem.getDefaultCommand() != null) {
            visionSubsystem.getDefaultCommand().cancel();
        }
    }

    /**
     * Executes the driving logic with AprilTag auto-aim enabled.
     */
    private void withCurrentDetection() {
        double rightX = driver.getRightX();

        // Execute the drive command with the combined rotation power
        driveSubsystem.teleDrive(driver.getLeftX(), driver.getLeftY(), visionSubsystem.getAutoAimPower2() * (1 - Math.abs(rightX)) + rightX,
                driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
    }
}
