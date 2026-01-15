package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.drive;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TELE_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TELE_DRIVE_POWER_TRIGGER_SCALE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.pickCustomPowerFunc;

/**
 * A field-centric tele-operated drive command.
 * <p>
 * This command provides a more intuitive driving experience by making the robot move
 * relative to the field's coordinate system, rather than its own orientation. Pushing the
 * left stick forward will always move the robot away from the driver, regardless of which
 * way the robot is facing.
 * <p>
 * How it works:
 * <ol>
 *     <li>At initialization, it captures the robot's starting heading.</li>
 *     <li>In the {@code execute} loop, it reads the joystick inputs, which are considered
 *     "field-centric" (e.g., Y is forward on the field, X is right on the field).</li>
 *     <li>It calculates the robot's current heading relative to its starting heading.</li>
 *     <li>It applies a rotation transformation to the joystick input vector to convert the desired
 *     field-centric movement into the robot's local coordinate frame.</li>
 *     <li>The resulting robot-centric velocities are sent to the drive subsystem.</li>
 * </ol>
 * The right stick still controls rotation as in a normal robot-centric drive.
 */
public class TeleDriveHybrid extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final GamepadWrapper driver;

    private double lastTeleDriverPowerScale = TELE_DRIVE_POWER;

    // Store initial heading for field-centric control
    private double initialHeading = 0.0;

    /**
     * Creates a new TeleDriveHybrid command.
     *
     * @param driveSubsystem   The drive subsystem to control.
     * @param gamepadSubsystem The gamepad subsystem for accessing driver inputs.
     */
    public TeleDriveHybrid(DriveSubsystem driveSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.driver = gamepadSubsystem.driver;

        addRequirements(driveSubsystem);
    }

    /**
     * Captures the robot's current heading as the reference for field-centric control.
     */
    @Override
    public void initialize() {
        resetFieldCentricHeading();
    }

    /**
     * Executes the field-centric drive logic.
     */
    @Override
    public void execute() {
        // Get joystick inputs
        double translationX = driver.getLeftX();
        double translationY = driver.getLeftY();
        double rotation = driver.getRightX();
        double powerScaler = driver.getRightTriggerValue();

        // Update power scale based on trigger
        if (powerScaler != 0) {
            lastTeleDriverPowerScale = TELE_DRIVE_POWER_TRIGGER_SCALE * pickCustomPowerFunc(powerScaler, 1)
                    + TELE_DRIVE_POWER;
        }

        // Get current robot heading and calculate relative heading from start
        double currentHeading = driveSubsystem.getPose().heading.toDouble();
        double relativeHeading = currentHeading - initialHeading;

        // Rotate the input vector by the negative of the relative heading to convert
        // from field-centric to robot-centric coordinates.
        double robotX = translationX * Math.cos(-relativeHeading) - translationY * Math.sin(-relativeHeading);
        double robotY = translationX * Math.sin(-relativeHeading) + translationY * Math.cos(-relativeHeading);

        // Apply the robot-centric velocities
        driveSubsystem.dinitechMecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(robotY, -robotX).times(lastTeleDriverPowerScale),
                -rotation * lastTeleDriverPowerScale));
    }

    /**
     * Resets the field-centric heading reference to the robot's current heading.
     * Call this to redefine which direction is "forward" for field-centric control.
     */
    public void resetFieldCentricHeading() {
        initialHeading = driveSubsystem.getPose().heading.toDouble();
    }

    /**
     * Stops the robot when the command ends.
     *
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.dinitechMecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}
