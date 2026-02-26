package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLAMPING_HEADING_ERROR;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.NUMBER_CUSTOM_POWER_FUNC_DRIVE_PEDRO_LOCKED;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.pickCustomPowerFunc;


import com.pedropathing.control.PIDFController;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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
public class PedroAimLockedDrive extends CommandBase {
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final GamepadWrapper driver;

    private final Supplier<Pose> goalPoseSupplier;
    private final PIDFController aimController;

    /**
     * Creates a new TeleDriveLocked command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     * @param gamepadSubsystem The gamepad subsystem for driver inputs.
     * @param goalPoseSupplier a supplier for the target Goal Pose
     */
    public PedroAimLockedDrive(DrivePedroSubsystem drivePedroSubsystem,
                               GamepadSubsystem gamepadSubsystem, Supplier<Pose> goalPoseSupplier) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.aimController = drivePedroSubsystem.getAimController();
        this.driver = gamepadSubsystem.getDriver();
        this.goalPoseSupplier = goalPoseSupplier;

        addRequirements(drivePedroSubsystem);
    }

    @Override
    public void initialize() {
        drivePedroSubsystem.setDriveAimLockType(DrivePedroSubsystem.DriveAimLockType.PEDRO_AIM);
    }

    /**
     * Switches between locked and normal drive modes based on AprilTag visibility.
     */
    @Override
    public void execute() {
        double rightX = driver.getRightX();
        Pose currentPose = drivePedroSubsystem.getPose().rotate(drivePedroSubsystem.getAccumulatedHeading(), false);

        Pose goalPose = goalPoseSupplier.get();

        double headingGoal = Math.atan2(currentPose.getY() - goalPose.getY(), currentPose.getX() - goalPose.getX());
        double headingError = MathFunctions.getTurnDirection(currentPose.getHeading(), headingGoal) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), headingGoal);
        double clampedError = Math.max(Math.min(headingError, CLAMPING_HEADING_ERROR), -CLAMPING_HEADING_ERROR);

        double powerCorrection = Math.pow(clampedError, NUMBER_CUSTOM_POWER_FUNC_DRIVE_PEDRO_LOCKED) * (1 - Math.abs(rightX));
        drivePedroSubsystem.teleDriveHybrid(driver.getLeftX(), driver.getLeftY(),powerCorrection + rightX, driver.getRightTriggerValue(), drivePedroSubsystem.getDriveReference() == DrivePedroSubsystem.DriveReference.FC);

//        aimController.updateError(headingError);
//        double correction = Math.min(Math.max(aimController.run(), -1), 1);

//        drivePedroSubsystem.teleDriveHybrid(driver.getLeftX(), driver.getLeftY(), Math.pow(correction, NUMBER_CUSTOM_POWER_FUNC_DRIVE_PEDRO_LOCKED) * (1 - Math.abs(rightX)) + rightX, driver.getRightTriggerValue(), drivePedroSubsystem.getDriveReference() == DrivePedroSubsystem.DriveReference.FC);


    }
}
