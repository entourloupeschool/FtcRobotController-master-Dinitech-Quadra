package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.DRIVER_POWER_SCALER_TO_THE_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SLOW_DRIVE_SCALE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TELE_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TELE_DRIVE_POWER_TRIGGER_SCALE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.pickCustomPowerFunc;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

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


    /**
     * Defines the operational state of the drive.
     */
    public enum DriveReference {
        Robot,
        FC
    }

    private DriveReference driveReference;

    /**
     * Sets the current reference state of the drive.
     * @param ref The new DriveUsageState.
     */
    public void setDriveReference(DriveReference ref) {
        this.driveReference = ref;
    }

    /**
     * Gets the current reference of the drive.
     * @return The current DriveReference
     */
    public DriveReference getDriveReference() {
        return driveReference;
    }


    /**
     * Defines the operational state of the drive.
     */
    public enum DriveUsage {
        TELE,   // Controlled by driver
        AIM_LOCKED, // Controlled by vision auto-aim
        SLOW,    // Slow drive mode
        BLOCKED // Lock in place
    }

    private DriveUsage driveUsage;

    /**
     * Sets the current usage state of the drive.
     * @param state The new DriveUsageState.
     */
    public void setDriveUsage(DriveUsage state) {
        this.driveUsage = state;
    }

    /**
     * Gets the current usage state of the drive.
     * @return The current DriveUsageState.
     */
    public DriveUsage getDriveUsage() {
        return driveUsage;
    }

    private double lastTeleDriverPowerScale = 1;

    /**
     * Constructs a new DriveSubsystem.
     *
     * @param hardwareMap         The hardware map for accessing robot hardware.
     * @param beginPose           The initial pose of the robot.
     * @param telemetry            The telemetry object for logging.
     */
    public DriveSubsystem(HardwareMap hardwareMap, Pose2d beginPose, final Telemetry telemetry) {
        this.dinitechMecanumDrive = new DinitechMecanumDrive(hardwareMap, beginPose);
        dinitechMecanumDrive.updatePoseEstimate();

        this.telemetry = telemetry;

    }

    /**
     * Drives the robot based on gamepad inputs with power scaling.
     *
     * @param translationX The strafing input, typically from a joystick's X-axis (-1 to 1).
     * @param translationY The forward/backward input, typically from a joystick's Y-axis (-1 to 1).
     * @param rotation     The rotational input, typically from another joystick's X-axis (-1 to 1).
     * @param powerScaler  A scaling factor for power, often from a trigger (0 to 1).
     */
    public void teleDriveHybrid(final double translationX, final double translationY, final double rotation,
                          final double powerScaler, boolean fieldCentric) {
        if (powerScaler != 0) {
            lastTeleDriverPowerScale = TELE_DRIVE_POWER_TRIGGER_SCALE * pickCustomPowerFunc(powerScaler, 1)
                    + TELE_DRIVE_POWER;
        }

        if (fieldCentric){
            this.fieldCentricTeleDrive(translationX * lastTeleDriverPowerScale, translationY * lastTeleDriverPowerScale, rotation * lastTeleDriverPowerScale);
            return;
        }

        dinitechMecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        translationY * lastTeleDriverPowerScale,
                        -translationX * lastTeleDriverPowerScale),
                -rotation * lastTeleDriverPowerScale));
    }

    /**
     * Drives the robot based on gamepad inputs with power scaling.
     *
     * @param translationX The strafing input, typically from a joystick's X-axis (-1 to 1).
     * @param translationY The forward/backward input, typically from a joystick's Y-axis (-1 to 1).
     * @param rotation     The rotational input, typically from another joystick's X-axis (-1 to 1).
     */
    public void fieldCentricTeleDrive(final double translationX, final double translationY, final double rotation) {
        this.getDrive().updatePoseEstimate();

        // Get the current robot heading
        Rotation2d botHeading = dinitechMecanumDrive.localizer.getPose().heading;

        // Create input vector from gamepad (robot-centric)
        Vector2d fieldInput = new Vector2d(
                translationY,
                -translationX);

        // Rotate the robot-centric input by the inverse of the bot heading
        // to convert to field-centric coordinates
        Vector2d robotInput = botHeading.inverse().times(fieldInput);

        dinitechMecanumDrive.setDrivePowers(new PoseVelocity2d(
                robotInput,
                -rotation));
    }

    public void stopAllMotors() {
        dinitechMecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    @Override
    public void periodic() {
        // This method is called periodically by the CommandScheduler.
        printDriveTelemetry(telemetry);
    }

    /**
     * Prints drive-related telemetry to the driver hub.
     * @param telemetry The telemetry object.
     */
    private void printDriveTelemetry(final Telemetry telemetry) {
        telemetry.addData("drive usage", driveUsage);
        telemetry.addData("drive reference", driveReference);

        Pose2d pose = getPose();
        telemetry.addLine("Robot Pose:");
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading", pose.heading.log());

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

    public void setLocalizer(Localizer localizer) {
        dinitechMecanumDrive.localizer = localizer;
    }

    public DinitechMecanumDrive getDrive(){return dinitechMecanumDrive;}

    /**
     * Gets the current pose (position and heading) of the robot.
     * @return The robot's current {@link Pose2d}.
     */
    public Pose2d getPose() {
        return getLocalizer().getPose();
    }

    public Telemetry getTelemetry(){
        return telemetry;
    }
}