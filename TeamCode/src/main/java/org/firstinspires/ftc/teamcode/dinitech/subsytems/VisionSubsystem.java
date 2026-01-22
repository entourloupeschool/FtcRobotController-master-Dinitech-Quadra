package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import com.acmerobotics.roadrunner.Pose2d;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_ORIENTATION_PITCH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_ORIENTATION_ROLL;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_ORIENTATION_YAW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_POSITION_X;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_POSITION_Y;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_POSITION_Z;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLAMP_BEARING;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CORRECTION_BASKET_OFFSET;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CX;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FX;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA1_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_RESOLUTION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.NUMBER_AT_SAMPLES;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.NUMBER_CUSTOM_POWER_FUNC_DRIVE_LOCKED;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.STREAM_FORMAT;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.aAT_LINE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.bAT_LINE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.cmToInch;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getLinearInterpolationOffsetBearing;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.pickCustomPowerFunc;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdatesAprilTagsDetections;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

// Unresolved error:
// https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/993

/**
 * A command-based subsystem that manages the robot's vision capabilities.
 * <p>
 * This subsystem handles the initialization and operation of a {@link VisionPortal},
 * which can be configured with multiple processors. It is designed to support both
 * AprilTag detection for localization and color detection for game-specific logic.
 * <p>
 * Key features:
 * <ul>
 *     <li>Manages an {@link AprilTagProcessor} for detecting AprilTags and estimating robot pose.</li>
 *     <li>Manages a {@link PredominantColorProcessor} for identifying object colors.</li>
 *     <li>Uses a running average filter to smooth AprilTag pose data, providing more stable localization.</li>
 *     <li>Dynamically adjusts AprilTag decimation to optimize performance based on distance.</li>
 *     <li>Caches game-specific information, like the randomized color order from setup tags.</li>
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {
    public final TelemetryManager telemetryM;
    public VisionPortal visionPortal;
    private boolean cameraStream = false;
    public AprilTagProcessor aprilTagProcessor;
    public PredominantColorProcessor colorProcessor;

    private int lastATPositionDetection = -1;
    private final RunningAverage robotPoseXCMSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage robotPoseYCMSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage robotPoseYDEGREESSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage rangeToAprilTagCMSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage cameraBearingDEGREESSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage confidenceAprilTagSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage aTPoseXCMSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage aTPoseYCMSamples = new RunningAverage(NUMBER_AT_SAMPLES);

    // Cached averages (updated via updateCachedAverages())
    private Double cachedRobotPoseXCM = null;
    private Double cachedRobotPoseYCM = null;
    private Double cachedRobotPoseYawDEGREES = null;
    private Double cachedRangeToAprilTagCM = null;
    private Double cachedCameraBearingDEGREES = null;
    private Double cachedConfidence = null;
    private Double cachedXATPoseCM = null;
    private Double cachedYATPoseCM = null;

    private boolean hasCurrentATDetections = false;

    private String[] cachedColorsOrder = new String[0];
    private boolean hasDetectedColorOrder = false;
    private double decimation = 1;

    public void setUsageState(VisionUsageState visionUsageState) {
        usageState = visionUsageState;
    }

    public VisionUsageState getUsageState() {
        return usageState;
    }

    /**
     * Defines the operational state of the shooter.
     */
    public enum VisionUsageState {
        CONTINUOUS,   // Continuous updates
        OPTIMIZED // Optimized updates
    }

    private VisionSubsystem.VisionUsageState usageState = VisionUsageState.OPTIMIZED;

    /**
     * Constructs a new VisionSubsystem.
     *
     * @param hardwareMap The robot's hardware map.
     * @param telemetryM   The telemetryM object for logging.
     */
    public VisionSubsystem(HardwareMap hardwareMap, final TelemetryManager telemetryM) {
        this.aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
//                .setDrawCubeProjection(true)
//                .setDrawAxes(true)
                .setCameraPose(
                        new Position(
                                DistanceUnit.CM,
                                CAMERA_POSITION_X, CAMERA_POSITION_Y, CAMERA_POSITION_Z, 0),
                        new YawPitchRollAngles(
                                AngleUnit.DEGREES,
                                CAMERA_ORIENTATION_YAW, CAMERA_ORIENTATION_PITCH, CAMERA_ORIENTATION_ROLL, 0))
                .setLensIntrinsics(FX, FY, CX, CY)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        this.visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA1_NAME))
                .setStreamFormat(STREAM_FORMAT)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setAutoStopLiveView(true)
                .enableLiveView(false)
                .build();

        this.setDefaultCommand(new ContinuousUpdatesAprilTagsDetections(this));

        this.telemetryM = telemetryM;
    }

    /**
     * Updates the cached AprilTag detection data.
     * <p>
     * If new detections are available, this method adds them to the running average filters
     * to provide smoothed pose and bearing information.
     */
    public void updateAprilTagDetections() {
        if (aprilTagProcessor != null) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            hasCurrentATDetections = !detections.isEmpty();

            if (hasCurrentATDetections) {
                AprilTagDetection detection = detections.get(0);

                if (detection.metadata != null) {
                    if (detection.id == 20 || detection.id == 24) {
                        lastATPositionDetection = detection.id;
                        robotPoseXCMSamples.add(detection.robotPose.getPosition().x);
                        robotPoseYCMSamples.add(detection.robotPose.getPosition().y);
                        robotPoseYDEGREESSamples.add(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                        cameraBearingDEGREESSamples.add(detection.ftcPose.bearing);
                        rangeToAprilTagCMSamples.add(detection.ftcPose.range);
                        confidenceAprilTagSamples.add(detection.decisionMargin);
                        aTPoseXCMSamples.add(detection.ftcPose.x);
                        aTPoseYCMSamples.add(detection.ftcPose.y);
                        updateCachedAverages();
                    } else if (!hasDetectedColorOrder) {
                        if (detection.id == 21) {
                            cachedColorsOrder = new String[] { "g", "p", "p" };
                            hasDetectedColorOrder = true;
                        } else if (detection.id == 22) {
                            cachedColorsOrder = new String[] { "p", "g", "p" };
                            hasDetectedColorOrder = true;
                        } else if (detection.id == 23) {
                            cachedColorsOrder = new String[] { "p", "p", "g" };
                            hasDetectedColorOrder = true;
                        }
                    }
                } else {
                    setHasCurrentAprilTagDetections(false);
                }
            }
        } else {
            setHasCurrentAprilTagDetections(false);
        }
    }

    /**
     * Updates all cached average values from the running average filters.
     * Call this after adding new samples to ensure cached values are current.
     */
    public void updateCachedAverages() {
        cachedRobotPoseXCM = robotPoseXCMSamples.getAverage();
        cachedRobotPoseYCM = robotPoseYCMSamples.getAverage();
        cachedRobotPoseYawDEGREES = robotPoseYDEGREESSamples.getAverage();
        cachedRangeToAprilTagCM = rangeToAprilTagCMSamples.getAverage();
        cachedCameraBearingDEGREES = cameraBearingDEGREESSamples.getAverage();
        cachedConfidence = confidenceAprilTagSamples.getAverage();
        cachedXATPoseCM = aTPoseXCMSamples.getAverage();
        cachedYATPoseCM = aTPoseYCMSamples.getAverage();
    }

    /**
     * A zero-allocation circular buffer for calculating a running average.
     */
    private static class RunningAverage {
        private final double[] samples;
        private int index = 0;
        private int count = 0;
        private double runningSum = 0;

        public RunningAverage(int size) {
            this.samples = new double[size];
        }

        public void add(double value) {
            runningSum -= samples[index];
            samples[index] = value;
            runningSum += value;
            index = (index + 1) % samples.length;
            if (count < samples.length) {
                count++;
            }
        }

        public Double getAverage() {
            return count == 0 ? null : runningSum / count;
        }

        public boolean isEmpty() {
            return count == 0;
        }

        public int size() {
            return count;
        }
    }

    /**
     * Gets the cached averaged robot pose X coordinate from AprilTag detections.
     *
     * @return The averaged X coordinate in inches, or null if no data is available.
     */
    public Double getRobotPoseX() {
        return cachedRobotPoseXCM;
    }

    /**
     * Gets the cached averaged robot pose Y coordinate from AprilTag detections.
     *
     * @return The averaged Y coordinate in inches, or null if no data is available.
     */
    public Double getRobotPoseY() {
        return cachedRobotPoseYCM;
    }

    /**
     * Gets the cached averaged robot yaw from AprilTag detections.
     *
     * @return The averaged yaw in radians, or null if no data is available.
     */
    public Double getRobotPoseYaw() {
        return cachedRobotPoseYawDEGREES;
    }

    /**
     * Gets the cached averaged range to the detected AprilTag.
     *
     * @return The averaged range in inches, or null if no data is available.
     */
    public Double getRangeToAprilTag() {
        return cachedRangeToAprilTagCM;
    }

    /**
     * Gets the cached averaged bearing from the camera to the AprilTag.
     *
     * @return The averaged bearing in degrees, or null if no data is available.
     */
    public Double getCameraBearing() {
        return cachedCameraBearingDEGREES;
    }

    /**
     * Calculates the angle between the robot's forward/backward axis and a line going from the center of the robot to the center of the AprilTag.
     * @param cameraBearing the camera's bearing from the AprilTag.
     * @param rangeToAprilTag the range to the AprilTag.
     * @return the corrected bearing in degrees.
     */
    public static double getRobotCenterToAprilTag(double cameraBearing, double rangeToAprilTag){
        return -cameraBearing + getLinearInterpolationOffsetBearing(rangeToAprilTag);
    }

    /**
     * Calculates the signed distance from the robot's center to a line perpendicular to the AprilTag.
     * Left of the AprilTag is positive, right is ,negative.
     * @param xRobot the robot's X coordinate relative to the field.
     * @param yRobot the robot's Y coordinate relative to the field.
     * @return the signed distance.
     */
    public static double getSignedDistanceToATLine(double xRobot, double yRobot, int tagID) {
        if (tagID == 20) return (yRobot - aAT_LINE * xRobot - bAT_LINE) / Math.sqrt(aAT_LINE * aAT_LINE + 1);
        else if (tagID == 24) return (yRobot + aAT_LINE * xRobot + bAT_LINE) / Math.sqrt(aAT_LINE * aAT_LINE + 1);
        else return 0;
    }

    /**
     * Computes a normalization and scales with range.
     * @param signedDistanceToATLine the signed distance from the robot's center to a line perpendicular to the AprilTag.
     * @param rangeToAT the range to the AprilTag.
     * @return the normalized correction.
     */
    public static double getNormalizedCorrectionWithRange(double signedDistanceToATLine, double rangeToAT) {
        return signedDistanceToATLine / (CORRECTION_BASKET_OFFSET * rangeToAT);
    }


    /**
     * Computes the auto-aim power based on the camera bearing, robot pose, and range to the AprilTag.
     * @return the auto-aim power for drive.
     */
    public double getAutoAimPower(){
        double cameraBearing = getCameraBearing() != null ? (double) getCameraBearing() : 0;
        double xRobot = getRobotPoseX() != null ? getRobotPoseX() : 0;
        double yRobot = getRobotPoseY() != null ? getRobotPoseY() : 0;
        double rangeToAT = getRangeToAprilTag() != null ? getRangeToAprilTag() : 50;

        double robotCenterBearing = getRobotCenterToAprilTag(cameraBearing, rangeToAT);
        double normalizedCorrectionWithRange = getNormalizedCorrectionWithRange(getSignedDistanceToATLine(xRobot, yRobot, lastATPositionDetection), rangeToAT);

        // Calculate the auto-aim rotation power from the bearing (sign preserving)
        return pickCustomPowerFunc(Math.max(-CLAMP_BEARING, Math.min(CLAMP_BEARING, robotCenterBearing + normalizedCorrectionWithRange)) / CLAMP_BEARING, NUMBER_CUSTOM_POWER_FUNC_DRIVE_LOCKED);
    }


//    public double getAutoAimPower3(){
//        double cameraBearing = getCameraBearing() != null ? (double) getCameraBearing() : 0;
//        double rangeToAprilTag = getRangeToAprilTag() != null ? (double) getRangeToAprilTag() : 0;
//
//
//        double l = Math.sqrt(-2 * CAMERA_POSITION_X * rangeToAprilTag * Math.cos(Math.toRadians(90 + cameraBearing)) + CAMERA_POSITION_X * CAMERA_POSITION_X * rangeToAprilTag * rangeToAprilTag);
//        double phi = Math.asin(Math.sin(Math.toRadians(90 + cameraBearing))/l);
//        double eps = Math.PI - phi - Math.toRadians(cameraBearing);
//        double theta = Math.PI/2 - Math.toRadians(cameraBearing);
//        double V = Math.PI/2 
//
//
//
//
//        return pickCustomPowerFunc(beta, NUMBER_CUSTOM_POWER_FUNC_DRIVE_LOCKED);
//    }

    /**
     * Gets the cached averaged XftcPose from AprilTag detections.
     *
     * @return The averaged XftcPose in inches, or null if no data is available.
     */
    public Double getXATPose(){
        return cachedXATPoseCM;
    }

    /**
     * Gets the cached averaged YftcPose from AprilTag detections.
     *
     * @return The averaged YftcPose in inches, or null if no data is available.
     */
    public Double getYATPose(){
        return cachedYATPoseCM;
    }


    /**
     * Checks if the last update cycle found any AprilTag detections.
     *
     * @return True if detections were found, false otherwise.
     */
    public boolean getHasCurrentAprilTagDetections() {
        return hasCurrentATDetections;
    }

    public void setHasCurrentAprilTagDetections(boolean newHasCurrentATDetections) {
        hasCurrentATDetections = newHasCurrentATDetections;
    }

    /**
     * Checks if there is any cached pose data from AprilTag detections.
     *
     * @return True if at least one pose sample has been collected.
     */
    public boolean hasCachedPoseData() {
        return !robotPoseXCMSamples.isEmpty();
    }

    /**
     * Gets the latest estimated robot pose from the averaged AprilTag data.
     *
     * @return A {@link Pose2d} representing the estimated robot pose.
     */
    public Pose2d getLatestRobotPoseEstimationFromAT() {
        Double x = getRobotPoseX();
        Double y = getRobotPoseY();
        Double yaw = getRobotPoseYaw();

        return new Pose2d(
                x != null ? cmToInch(x) : 0.0,
                y != null ? cmToInch(y) : 0.0,
                yaw != null ? yaw : 0.0);
    }


    private void setAprilTagDetectionDecimation(double dec) {
        aprilTagProcessor.setDecimation((float) dec);
    }

    /**
     * Dynamically optimizes the AprilTag processor's decimation based on the range to the tag.
     * This balances detection range and processing rate.
     */
    public void optimizeDecimation() {
        if (aprilTagProcessor == null) return;
        
        Double rangeToAprilTag = getRangeToAprilTag();
        if (rangeToAprilTag == null) return;

        double optimalDecimation = 4.0 - 3.0 * (rangeToAprilTag - 35) / 45.0;
        optimalDecimation = Math.max(1, Math.min(4, optimalDecimation));
        
        setDecimation(optimalDecimation);

    }

    public double getDecimation(){
        return decimation;
    }
    
    public void setDecimation(double newDecimation){
        decimation = newDecimation;
        setAprilTagDetectionDecimation(decimation);
    }

    @Override
    public void periodic() {
        aprilTagProcessorTelemetryManager();
        telemetryM.addData("vision usage state", getUsageState());
    }


    private void aprilTagProcessorTelemetryManager() {
        telemetryM.addData("Current AT Detections", getHasCurrentAprilTagDetections() ? "Yes" : "No");

        if (hasCachedPoseData()) {
            telemetryM.addLine("last detected pose values");
            telemetryM.addData("X Robot (CM)", getRobotPoseX());
            telemetryM.addData("Y Robot (CM)",  getRobotPoseY());
//            telemetryM.addData("Yaw (DEGREES)", "%.2f", getRobotPoseYaw());
//            telemetryM.addData("Camera Bearing (DEGREES)", "%.2f", getCameraBearing());
//            telemetryM.addData("Robot Center Bearing", "%.2f", getRobotCenterBearing());
//            telemetryM.addData("Range (CM)", "%.2f", getRangeToAprilTag());
            telemetryM.addData("X AT (CM)", getXATPose());
            telemetryM.addData("Y AT (CM)", getYATPose());
//            telemetryM.addData("robotCenterBearing", getRobotCenterToAprilTag(getCameraBearing(), getRangeToAprilTag()));
//            telemetryM.addData("normalizedCorrectionWithRange", "%.2f", getNormalizedCorrectionWithRange(getSignedDistanceToATLine(getRobotPoseX(), getRobotPoseY()), getRangeToAprilTag()));
        } else {
            telemetryM.addData("AprilTag Pose Data", "No sample data");
        }

        telemetryM.addData("Decimation", getDecimation());

        if (hasDetectedColorOrder) {
            telemetryM.addData("Artifact Color Order", String.join(", ", cachedColorsOrder));
        }
    }

    private void colorProcessorTelemetryManager() {
        PredominantColorProcessor.Result result = colorProcessor.getAnalysis();
        telemetryM.addData("Predominant Color", result != null ? result.closestSwatch : "No analysis");
    }

    /**
     * Gets the cached color order determined at the start of the match.
     *
     * @return An array of strings representing the color order (e.g., ["g", "p", "p"]), or an empty array if not yet detected.
     */
    public String[] getColorsOrder() {
        return cachedColorsOrder;
    }

    /**
     * Checks if the color order has been detected and cached.
     *
     * @return True if the color order is available, false otherwise.
     */
    public boolean hasColorOrder() {
        return hasDetectedColorOrder;
    }

    /**
     * Resets the cached color order.
     */
    public void resetColorOrder() {
        cachedColorsOrder = new String[0];
        hasDetectedColorOrder = false;
    }

    /**
     * toggle AprilTag processor enabled/disabled.
     */
    public void setAprilTagProcessorEnabled(boolean newCameraStream){
        if (visionPortal == null) return;

        visionPortal.setProcessorEnabled(aprilTagProcessor, newCameraStream);
    }

    /**
     * get AprilTag processor enabled/disabled.
     */
    public boolean getAprilTagProcessorEnabled(){
        if (visionPortal == null) return false;

        return visionPortal.getProcessorEnabled(aprilTagProcessor);
    }



    public VisionPortal getVisionPortal(){
        if (visionPortal == null) return null;

        return visionPortal;
    }

    public void turnCameraOff(){
        visionPortal.stopStreaming();
    }

    public void turnCameraOn(){
        visionPortal.resumeStreaming();
    }
}
