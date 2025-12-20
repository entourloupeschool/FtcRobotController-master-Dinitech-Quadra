package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DRIVER_POWER_SCALER_TO_THE_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SLOW_DRIVE_SCALE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TELE_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TELE_DRIVE_POWER_TRIGGER_SCALE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.pickCustomPowerFunc;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechMecanumDrive;

import java.util.HashSet;
import java.util.Set;

/**
 * A command-based subsystem that manages the robot's {@link DinitechMecanumDrive}.
 * <p>
 * This class serves as a bridge between high-level robot commands and the underlying
 * drive hardware. It provides methods for tele-operated driving with various speed
 * scaling options and can be integrated into a command-based robot structure.
 *
 * @see DinitechMecanumDrive
 * @see SubsystemBase
 */
public class DriveSubsystem extends SubsystemBase {
    /** The core drive system with Road Runner integration. */
    public DinitechMecanumDrive dinitechMecanumDrive;
    /** Telemetry for reporting drive status. */
    private final Telemetry telemetry;

    private boolean isATLocked = false;
    private boolean isSlowDrive = false;

    private double lastTeleDriverPowerScale = TELE_DRIVE_POWER;

    /**
     * Constructs a new DriveSubsystem.
     *
     * @param dinitechMecanumDrive The configured {@link DinitechMecanumDrive} instance.
     * @param telemetry            The telemetry object for logging.
     */
    public DriveSubsystem(final DinitechMecanumDrive dinitechMecanumDrive, final Telemetry telemetry) {
        this.dinitechMecanumDrive = dinitechMecanumDrive;
        this.telemetry = telemetry;
        dinitechMecanumDrive.updatePoseEstimate();
    }

    /**
     * Drives the robot based on gamepad inputs with power scaling.
     *
     * @param translationX The strafing input, typically from a joystick's X-axis (-1 to 1).
     * @param translationY The forward/backward input, typically from a joystick's Y-axis (-1 to 1).
     * @param rotation     The rotational input, typically from another joystick's X-axis (-1 to 1).
     * @param powerScaler  A scaling factor for power, often from a trigger (0 to 1).
     */
    public void teleDrive(final double translationX, final double translationY, final double rotation,
            final double powerScaler) {
        if (powerScaler != 0) {
            lastTeleDriverPowerScale = TELE_DRIVE_POWER_TRIGGER_SCALE * pickCustomPowerFunc(powerScaler, 1)
                    + TELE_DRIVE_POWER;
        }
        dinitechMecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        translationY * lastTeleDriverPowerScale,
                        -translationX * lastTeleDriverPowerScale),
                -rotation * lastTeleDriverPowerScale));
    }

    /**
     * Drives the robot at a reduced, fixed speed for precise maneuvering.
     *
     * @param translationX The strafing input (-1 to 1).
     * @param translationY The forward/backward input (-1 to 1).
     * @param rotation     The rotational input (-1 to 1).
     * @param powerScaler  (Not used) A scaling factor for power.
     */
    public void teleSlowDrive(final double translationX, final double translationY, final double rotation,
            final double powerScaler) {

        dinitechMecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        translationY * SLOW_DRIVE_SCALE,
                        -translationX * SLOW_DRIVE_SCALE),
                -rotation * SLOW_DRIVE_SCALE));
    }

    /**
     * Drives the robot with direct, unscaled power from the gamepad inputs.
     *
     * @param translationX The strafing input (-1 to 1).
     * @param translationY The forward/backward input (-1 to 1).
     * @param rotation     The rotational input (-1 to 1).
     * @param powerScaler  (Not used) A scaling factor for power.
     */
    public void teleDriveUnscale(final double translationX, final double translationY, final double rotation,
            final double powerScaler) {
        dinitechMecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        translationY,
                        -translationX),
                -rotation));
    }

    /**
     * Sets the AprilTag lock state.
     * @param newIsATLocked The new lock state.
     */
    public void setIsATLocked(boolean newIsATLocked) {
        isATLocked = newIsATLocked;
    }

    /**
     * Gets the AprilTag lock state.
     * @return The current lock state.
     */
    public boolean getIsATLocked() {
        return isATLocked;
    }

    @Override
    public void periodic() {
        // This method is called periodically by the CommandScheduler.
    }

    /**
     * Prints drive-related telemetry to the driver hub.
     * @param telemetry The telemetry object.
     */
    private void printDriveTelemetry(final Telemetry telemetry) {

    }

    /**
     * Returns a set containing this subsystem, for use in command requirements.
     * @return A set containing the DriveSubsystem.
     */
    public Set<Subsystem> getDriveSubsystemSet() {
        Set<Subsystem> setDriveSubsystem = new HashSet<Subsystem>();
        setDriveSubsystem.add(this);
        return setDriveSubsystem;
    }

    /**
     * Gets the localizer from the underlying mecanum drive.
     * @return The current {@link Localizer}.
     */
    public Localizer getLocalizer() {
        return dinitechMecanumDrive.localizer;
    }

    /**
     * Gets the current pose (position and heading) of the robot.
     * @return The robot's current {@link Pose2d}.
     */
    public Pose2d getPose() {
        return getLocalizer().getPose();
    }

    /**
     * Checks if the drive is currently in slow mode.
     * @return True if in slow mode, false otherwise.
     */
    public boolean isSlowDrive() {
        return isSlowDrive;
    }

    /**
     * Sets the slow drive mode.
     * @param slowDrive True to enable slow drive, false to disable.
     */
    public void setIsSlowDrive(boolean slowDrive) {
        isSlowDrive = slowDrive;
    }

    /**
     * Builds a short, field-centric trajectory action for tele-operated control.
     * <p>
     * This allows for precise, non-blocking movements relative to the field, where the
     * left stick controls translation and the right stick controls rotation.
     *
     * @param velocityX  The desired X velocity in the field frame (-1 to 1, left stick X).
     * @param velocityY  The desired Y velocity in the field frame (-1 to 1, left stick Y).
     * @param rotation   The desired rotational velocity (-1 to 1, right stick X).
     * @param duration   The duration of the trajectory in seconds.
     * @return A Road Runner {@link Action} for following the generated trajectory.
     */
    public Action buildTeleopTrajectoryAction(double velocityX, double velocityY,
            double rotation, double duration) {
        // Get current pose
        Pose2d currentPose = dinitechMecanumDrive.localizer.getPose();

        // Scale inputs to reasonable velocities (in inches per second for translation)
        double maxTranslationSpeed = 30.0; // inches per second
        double maxRotationSpeed = Math.PI; // radians per second

        // FIELD-CENTRIC: Calculate displacement directly in world frame
        // Left stick Y -> Forward on field (positive Y)
        // Left stick X -> Strafe right on field (positive X)
        double worldDeltaX = -velocityX * maxTranslationSpeed * duration; // Strafe (left stick X)
        double worldDeltaY = velocityY * maxTranslationSpeed * duration; // Forward (left stick Y)
        double deltaHeading = -rotation * maxRotationSpeed * duration; // Rotation (right stick X)

        // Calculate target position in world frame
        Vector2d targetPosition = new Vector2d(
                currentPose.position.x + worldDeltaX,
                currentPose.position.y + worldDeltaY);

        // Calculate target heading
        double targetHeading = currentPose.heading.toDouble() + deltaHeading;

        // Build trajectory action with higher constraint scale for responsive teleop
        return dinitechMecanumDrive.actionBuilder(currentPose, 1.5)
                .strafeTo(targetPosition)
                .turnTo(targetHeading)
                .build();
    }
}