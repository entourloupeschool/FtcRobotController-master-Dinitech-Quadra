package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TELE_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TELE_DRIVE_POWER_TRIGGER_SCALE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.pickCustomPowerFunc;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.dinitech.other.DrawingDinitech;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechPedroMecanumDrive;

public class DrivePedroSubsystem extends SubsystemBase {
    /** The core drive system with PedroPathing integration. */
    public DinitechPedroMecanumDrive dinitechPedroMecanumDrive;
    /** Telemetry for reporting drive status. */
    private final TelemetryManager telemetryM;

    public boolean followerIsBusy() {
        return dinitechPedroMecanumDrive.isBusy();
    }

    public boolean isPathQuasiDone(){
        return dinitechPedroMecanumDrive.isPathQuasiDone();
    }


    public void setMaxPower(double globalMaxPower) {
        dinitechPedroMecanumDrive.setMaxPower(globalMaxPower);
    }


    /**
     * Defines the operational state of the drive.
     */
    public enum DriveReference {
        ROBOT,
        FC,
        AUTO
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
        BLOCKED, // Lock in place
        AUTO // Controlled by autonomous code
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
     * @param telemetryM            The telemetry object for logging.
     */
    public DrivePedroSubsystem(HardwareMap hardwareMap, Pose beginPose, final TelemetryManager telemetryM) {
        this.dinitechPedroMecanumDrive = new DinitechPedroMecanumDrive(hardwareMap, beginPose);

        setDriveUsage(DriveUsage.AUTO);
        setDriveReference(DriveReference.FC);
        this.telemetryM = telemetryM;

    }

    public DrivePedroSubsystem(HardwareMap hardwareMap, final TelemetryManager telemetryM) {
        this.dinitechPedroMecanumDrive = new DinitechPedroMecanumDrive(hardwareMap);

        setDriveUsage(DriveUsage.AUTO);
        setDriveReference(DriveReference.FC);
        this.telemetryM = telemetryM;

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

        dinitechPedroMecanumDrive.setDrivePowers(
                        translationY * lastTeleDriverPowerScale,
                        -translationX * lastTeleDriverPowerScale,
                -rotation * lastTeleDriverPowerScale,
                !fieldCentric);
    }

    public void stopAllMotors() {
        dinitechPedroMecanumDrive.setDrivePowers(0, 0, 0, false);
    }

    public void followPathChain(PathChain pathChain, double maxPower, boolean holdEnd){
        dinitechPedroMecanumDrive.followPathChain(pathChain, maxPower, holdEnd);
    }

    @Override
    public void periodic() {
        dinitechPedroMecanumDrive.update();
        // This method is called periodically by the CommandScheduler.
//        printDriveTelemetry(telemetryM);
//        debugPedro(telemetryM);
    }

    private void debugPedro(TelemetryManager telemetryM) {
        if (dinitechPedroMecanumDrive.isOnPath()){
            telemetryM.addData("quasi", isPathQuasiDone());
            telemetryM.addData("done", !followerIsBusy());
        }

        DrawingDinitech.drawDebug(dinitechPedroMecanumDrive.getFollower());
    }

    /**
     * Prints drive-related telemetry to the driver hub.
     * @param telemetryM The telemetry object.
     */
    private void printDriveTelemetry(final TelemetryManager telemetryM) {
        telemetryM.addData("drive usage", getDriveUsage());
        telemetryM.addData("drive reference", getDriveReference());
//        telemetryM.addData("tFollower", dinitechPedroMecanumDrive.getFollower().getCurrentTValue());

//        Pose pose = getPose();
//        telemetryM.addLine("Robot Pose:");
//        telemetryM.addData("x", pose.getX());
//        telemetryM.addData("y", pose.getY());
//        telemetryM.addData("heading", pose.getHeading());

    }

    public DinitechPedroMecanumDrive getDrive(){return dinitechPedroMecanumDrive;}

    /**
     * Gets the current pose (position and heading) of the robot.
     * @return The robot's current {@link Pose}.
     */
    public Pose getPose() {
        return dinitechPedroMecanumDrive.getPose();
    }

    /**
     * Gets the current heading of the robot.
     * @return The robot's current heading
     */
    public double getHeading() {
        return dinitechPedroMecanumDrive.getHeading();
    }

    public void setHeading(double heading){
        dinitechPedroMecanumDrive.setHeading(heading);
    }

    public TelemetryManager getTelemetry(){
        return telemetryM;
    }
}