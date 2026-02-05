package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.drivePedro;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BLUE_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PEDRO_AIMING_CONTROLLER_D;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PEDRO_AIMING_CONTROLLER_F;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PEDRO_AIMING_CONTROLLER_I;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.PEDRO_AIMING_CONTROLLER_P;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RED_BASKET_POSE;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

import java.util.function.BooleanSupplier;

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
    private BooleanSupplier isBlue;
    private Pose goalPose;
    private PIDFController controller;
    /**
     * Creates a new TeleDriveLocked command.
     *
     * @param drivePedroSubsystem   The drive subsystem to control.
     * @param gamepadSubsystem The gamepad subsystem for driver inputs.
     * @param isBlue robot's team
     */
    public PedroAimLockedDrive(DrivePedroSubsystem drivePedroSubsystem,
                               GamepadSubsystem gamepadSubsystem, BooleanSupplier isBlue) {
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.driver = gamepadSubsystem.getDriver();
        this.isBlue = isBlue;

        addRequirements(drivePedroSubsystem);
    }

    @Override
    public void initialize() {
        drivePedroSubsystem.setDriveAimLockType(DrivePedroSubsystem.DriveAimLockType.PEDRO_AIM);
        controller = new PIDFController(new PIDFCoefficients(PEDRO_AIMING_CONTROLLER_P, PEDRO_AIMING_CONTROLLER_I,PEDRO_AIMING_CONTROLLER_D, PEDRO_AIMING_CONTROLLER_F));

        goalPose = isBlue.getAsBoolean() ? BLUE_BASKET_POSE : RED_BASKET_POSE;
        drivePedroSubsystem.teleDriveHybrid(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), 1, drivePedroSubsystem.getDriveReference() == DrivePedroSubsystem.DriveReference.FC);
    }

    /**
     * Switches between locked and normal drive modes based on AprilTag visibility.
     */
    @Override
    public void execute() {
        goalPose = isBlue.getAsBoolean() ? BLUE_BASKET_POSE : RED_BASKET_POSE;
        controller.setCoefficients(new PIDFCoefficients(PEDRO_AIMING_CONTROLLER_P, PEDRO_AIMING_CONTROLLER_I,PEDRO_AIMING_CONTROLLER_D, PEDRO_AIMING_CONTROLLER_F));

        double rightX = driver.getRightX();
        Pose currentPose = drivePedroSubsystem.getPose();

        double headingGoal = Math.atan2(currentPose.getY() - goalPose.getY(), currentPose.getX() - goalPose.getX());
        double headingError = MathFunctions.getTurnDirection(currentPose.getHeading(), headingGoal) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), headingGoal);

        controller.updateError(headingError);

        drivePedroSubsystem.teleDriveHybrid(driver.getLeftX(), driver.getLeftY(), controller.run() * (1 - Math.abs(rightX)) + rightX, driver.getRightTriggerValue(), drivePedroSubsystem.getDriveReference() == DrivePedroSubsystem.DriveReference.FC);
    }
}
