package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.AutoPathsDefinitions.FOLLOWER_T_POSITION_END;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.pickCustomPowerFunc;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.dinitech.other.DrawingDinitech;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechPredictiveFollower;

@Configurable
public class DrivePedroSubsystem extends SubsystemBase {
    public static final double TELE_DRIVE_POWER = 0.3;
    public static final double TELE_DRIVE_POWER_TRIGGER_SCALE = 1 - TELE_DRIVE_POWER;
    public static final double SLOW_DRIVE_SCALE = 0.3;
    public static final double THROUGH_BORE_ENCODER_COUNTS_PER_REV = 8192; // https://revrobotics.eu/rev-11-1271/
    public static final double DEAD_WHEEL_DIAMETER_MM = 50.8; // https://revrobotics.eu/ION-Omni-Wheels/
    public static final double ENCODER_RESOLUTION = THROUGH_BORE_ENCODER_COUNTS_PER_REV / (DEAD_WHEEL_DIAMETER_MM * Math.PI);
    public static final double PAR_POD_Y_MM = -142.2; // -144.5;
    public static final double PERP_POD_X_MM = 143.2; //147;//143;
    public static final double CLAMPING_HEADING_ERROR = 0.39;
    public static final int NUMBER_CUSTOM_POWER_FUNC_DRIVE_PEDRO_LOCKED = 3;
    public static double SCALE_HEADING_CONSTRAINT_TELEOP = 0.1;

    private final Follower follower;
    private double followerTEnd = FOLLOWER_T_POSITION_END;

    public void setFollowerTEnd(double followerTEnd) {
        this.followerTEnd = followerTEnd;
    }

    public double getFollowerTEnd(){
        return followerTEnd;
    }


    /**
     * Telemetry for reporting drive status.
     */
    private final TelemetryManager telemetryM;

    private boolean driverInputPose;

    public void setDriverInputPose(boolean driverInputPose) {
        this.driverInputPose = driverInputPose;
    }

    /**
     * Wether the driver has input a pose for the robot.
     *
     * @return True if the driver has input a pose, false otherwise.
     */
    public boolean getDriverInputPose() {
        return driverInputPose;
    }


    public boolean followerIsBusy() {
        return follower.isBusy();
    }

    public boolean isPathQuasiDone() {
        if (followerTEnd <= FOLLOWER_T_POSITION_END) return follower.getCurrentTValue() > followerTEnd;

        return follower.getCurrentTValue() >= followerTEnd && follower.getHeadingError() < follower.getCurrentPath().getPathEndHeadingConstraint()*SCALE_HEADING_CONSTRAINT_TELEOP;
    }

    public void pausePathFollowing() {
        follower.pausePathFollowing();
    }

    public void resumePathFollowing() {
        follower.resumePathFollowing();
    }

    public void startTeleOpDrive(boolean b) {
        follower.startTeleOpDrive();
    }

    public PathBuilder getPathBuilder() {
        return follower.pathBuilder();
    }

    public void prepAuto(Pose pose) {
        follower.setStartingPose(pose);
        follower.update();
    }


    /**
     * Defines the operational state of the drive.
     */
    public enum DriveReference {
        ROBOT,
        FC
    }

    private DriveReference driveReference;

    /**
     * Sets the current reference state of the drive.
     *
     * @param ref The new DriveUsageState.
     */
    public void setDriveReference(DriveReference ref) {
        this.driveReference = ref;
    }

    /**
     * Gets the current reference of the drive.
     *
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
        BLOCKED, // Lock in place
        AUTO // Controlled by autonomous code
    }

    private DriveUsage driveUsage;

    /**
     * Sets the current usage state of the drive.
     *
     * @param state The new DriveUsageState.
     */
    public void setDriveUsage(DriveUsage state) {
        this.driveUsage = state;
    }

    /**
     * Gets the current usage state of the drive.
     *
     * @return The current DriveUsageState.
     */
    public DriveUsage getDriveUsage() {
        return driveUsage;
    }

    public enum DriveAimLockType {
        NONE, // No drive lock
        VISION_AIM, // vision apriltag values
        PEDRO_AIM // pedro poses
    }

    private DriveAimLockType driveAimLockType;


    public DriveAimLockType getDriveAimLockType() {
        return driveAimLockType;
    }

    public void setDriveAimLockType(DriveAimLockType driveAimLockType) {
        this.driveAimLockType = driveAimLockType;
    }

    private double lastTeleDriverPowerScale = 1;

    public double getLastTeleDriverPowerScale() {
        return lastTeleDriverPowerScale;
    }

    public void setLastTeleDriverPowerScale(double lastTeleDriverPowerScale) {
        this.lastTeleDriverPowerScale = lastTeleDriverPowerScale;
    }

    /**
     * Constructs a new DriveSubsystem.
     *
     * @param hardwareMap The hardware map for accessing robot hardware.
     * @param beginPose   The initial pose of the robot.
     * @param telemetryM  The telemetry object for logging.
     */
    public DrivePedroSubsystem(HardwareMap hardwareMap, Pose beginPose, final TelemetryManager telemetryM) {
        this.follower = DinitechPredictiveFollower.createFollower(hardwareMap);
        follower.setStartingPose(beginPose);
        follower.update();

        setDriverInputPose(false);
        setDriveUsage(DriveUsage.TELE);
        setDriveReference(DriveReference.FC);
        setDriveAimLockType(DriveAimLockType.NONE);
        setLastTeleDriverPowerScale(1);

        this.telemetryM = telemetryM;
    }

    public DrivePedroSubsystem(HardwareMap hardwareMap, final TelemetryManager telemetryM) {
        this.follower = DinitechPredictiveFollower.createFollower(hardwareMap);

        setDriverInputPose(false);
        setDriveUsage(DriveUsage.TELE);
        setDriveReference(DriveReference.FC);
        setDriveAimLockType(DriveAimLockType.NONE);
        setLastTeleDriverPowerScale(1);

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
        if (powerScaler >= 0.02) {
            setLastTeleDriverPowerScale(TELE_DRIVE_POWER_TRIGGER_SCALE * pickCustomPowerFunc(1 - powerScaler, 1)
                    + TELE_DRIVE_POWER);
        }

        double lastPowerScale = getLastTeleDriverPowerScale();

        follower.setTeleOpDrive(
                translationY * lastPowerScale,
                -translationX * lastPowerScale,
                -rotation * lastPowerScale,
                !fieldCentric);
    }

    public void stopAllMotors() {
        follower.setTeleOpDrive(0, 0, 0, false);
    }

    public void followPathChain(PathChain pathChain, double maxPower, boolean holdEnd) {
        follower.followPath(pathChain, maxPower, holdEnd);
    }

    @Override
    public void periodic() {
        follower.update();
        // This method is called periodically by the CommandScheduler.
//        printDriveTelemetry(telemetryM);
//        debugPedro(telemetryM);
    }

    private void debugPedro(TelemetryManager telemetryM) {
        if (follower.getCurrentPath() != null) {
            telemetryM.addData("quasi", isPathQuasiDone());
            telemetryM.addData("done", !followerIsBusy());
        }

        DrawingDinitech.drawDebug(follower);
    }

    /**
     * Prints drive-related telemetry to the driver hub.
     *
     * @param telemetryM The telemetry object.
     */
    private void printDriveTelemetry(final TelemetryManager telemetryM) {
        telemetryM.addData("drive usage", getDriveUsage());
//        telemetryM.addData("drive reference", getDriveReference());
        telemetryM.addData("aimLockType", getDriveAimLockType());

//        Pose pose = getPose();
//        telemetryM.addLine("Robot Pose:");
//        telemetryM.addData("x", pose.getX());
//        telemetryM.addData("y", pose.getY());
//        telemetryM.addData("heading", pose.getHeading());
//        telemetryM.addData("distance", pose.distanceFrom(ROTATED_BLUE_BASKET_POSE));
    }

    /**
     * Gets the current pose (position and heading) of the robot.
     *
     * @return The robot's current {@link Pose}.
     */
    public Pose getPose() {
        return follower.getPose();
    }

    public void setPose(Pose newPose){
        follower.setPose(newPose);
    }

    /**
     * Gets the current heading of the robot.
     *
     * @return The robot's current heading
     */
    public double getHeading() {
        return follower.getHeading();
    }

    public void setHeading(double heading) {
        follower.setHeading(heading);
    }

    public TelemetryManager getTelemetry() {
        return telemetryM;
    }
}
