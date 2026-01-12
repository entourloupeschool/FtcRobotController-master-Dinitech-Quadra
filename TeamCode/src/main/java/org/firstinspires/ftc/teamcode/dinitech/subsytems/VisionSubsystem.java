package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BASKET_Y_OFFSET;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_ORIENTATION_PITCH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_ORIENTATION_ROLL;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_ORIENTATION_YAW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_POSITION_X;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_POSITION_Y;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_POSITION_Z;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLAMP_BEARING;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CX;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FX;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA1_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_RESOLUTION;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.NUMBER_AT_SAMPLES;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.NUMBER_CUSTOM_POWER_FUNC_DRIVE_LOCKED;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SCALER_OFFSET_AT_TO_X_BASKET;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.STREAM_FORMAT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.USE_WEBCAM;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.getLinearInterpolationOffsetBearing;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.pickCustomPowerFunc;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

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
    public final Telemetry telemetry;
    public VisionPortal.Builder builder = null;
    public AprilTagProcessor aprilTagProcessor;
    public PredominantColorProcessor colorProcessor;

    private final RunningAverage robotPoseXSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage robotPoseYSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage robotPoseYawSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage rangeToAprilTagSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage cameraBearingSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage confidenceAprilTagSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage aTPoseXSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage aTPoseYSamples = new RunningAverage(NUMBER_AT_SAMPLES);

    // Cached averages (updated via updateCachedAverages())
    private Double cachedRobotPoseX = null;
    private Double cachedRobotPoseY = null;
    private Double cachedRobotPoseYaw = null;
    private Double cachedRangeToAprilTag = null;
    private Double cachedCameraBearing = null;
    private Double cachedConfidence = null;
    private Double cachedXATPose = null;
    private Double cachedYATPose = null;

    private boolean hasCurrentATDetections = false;

    private String[] cachedColorsOrder = new String[0];
    private boolean hasDetectedColorOrder = false;
    private int decimation = 1;

    /**
     * Constructs a new VisionSubsystem.
     *
     * @param hardwareMap The robot's hardware map.
     * @param telemetry   The telemetry object for logging.
     */
    public VisionSubsystem(HardwareMap hardwareMap, final Telemetry telemetry) {
        this.builder = new VisionPortal.Builder();
        this.aprilTagProcessor = null;
        this.colorProcessor = null;

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, CAMERA1_NAME))
                    .setStreamFormat(STREAM_FORMAT)
                    .setCameraResolution(CAMERA_RESOLUTION)
                    .enableLiveView(false);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        this.telemetry = telemetry;

        builder.addProcessor(getAprilTagProcessor());
        builder.build();
    }

    /**
     * Adds the AprilTag processor to the VisionPortal builder.
     */
    public AprilTagProcessor getAprilTagProcessor() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setDrawCubeProjection(true)
                .setDrawAxes(true)
                .setCameraPose(
                        new Position(
                                DistanceUnit.CM,
                                CAMERA_POSITION_X, CAMERA_POSITION_Y, CAMERA_POSITION_Z, 0
                        ), new YawPitchRollAngles(
                                AngleUnit.DEGREES,
                                CAMERA_ORIENTATION_YAW, CAMERA_ORIENTATION_PITCH, CAMERA_ORIENTATION_ROLL, 0
                        ))
                .setLensIntrinsics(FX, FY, CX, CY)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        setAprilTagDetectionDecimation(getDecimation());

        return aprilTagProcessor;
    }

    /**
     * Adds the color processor to the VisionPortal builder.
     *
     * @param size The size of the region of interest for color analysis.
     */
    public void addColorProcessor(final double size) {
        colorProcessor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-size, size, size, -size))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE)
                .build();

        builder.addProcessor(colorProcessor);
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
                        robotPoseXSamples.add(detection.robotPose.getPosition().x);
                        robotPoseYSamples.add(detection.robotPose.getPosition().y);
                        robotPoseYawSamples.add(detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
                        cameraBearingSamples.add(detection.ftcPose.bearing);
                        rangeToAprilTagSamples.add(detection.ftcPose.range);
                        confidenceAprilTagSamples.add(detection.decisionMargin);
                        aTPoseXSamples.add(detection.ftcPose.x);
                        aTPoseYSamples.add(detection.ftcPose.y);
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
        cachedRobotPoseX = robotPoseXSamples.getAverage();
        cachedRobotPoseY = robotPoseYSamples.getAverage();
        cachedRobotPoseYaw = robotPoseYawSamples.getAverage();
        cachedRangeToAprilTag = rangeToAprilTagSamples.getAverage();
        cachedCameraBearing = cameraBearingSamples.getAverage();
        cachedConfidence = confidenceAprilTagSamples.getAverage();
        cachedXATPose = aTPoseXSamples.getAverage();
        cachedYATPose = aTPoseYSamples.getAverage();
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
        return cachedRobotPoseX;
    }

    /**
     * Gets the cached averaged robot pose Y coordinate from AprilTag detections.
     *
     * @return The averaged Y coordinate in inches, or null if no data is available.
     */
    public Double getRobotPoseY() {
        return cachedRobotPoseY;
    }

    /**
     * Gets the cached averaged robot yaw from AprilTag detections.
     *
     * @return The averaged yaw in radians, or null if no data is available.
     */
    public Double getRobotPoseYaw() {
        return cachedRobotPoseYaw;
    }

    /**
     * Gets the cached averaged range to the detected AprilTag.
     *
     * @return The averaged range in inches, or null if no data is available.
     */
    public Double getRangeToAprilTag() {
        return cachedRangeToAprilTag;
    }

    /**
     * Gets the cached averaged bearing from the camera to the AprilTag.
     *
     * @return The averaged bearing in degrees, or null if no data is available.
     */
    public Double getCameraBearing() {
        return cachedCameraBearing;
    }

    public double getRobotCenterBearing(){
        double cameraBearing = -getCameraBearing();
        return cameraBearing - getLinearInterpolationOffsetBearing(getRangeToAprilTag());
    }

    public double getNormalizedClampedRobotCenterBasketBearing(){
        // Invert the camera bearing because the robot's rotation direction is opposite
        // to the bearing's sign (e.g., a positive bearing requires a negative rotation).
        double invertedCameraBearing = -getCameraBearing();

        double range = getRangeToAprilTag();

        // Simple add the camera sideway : allways negative because  camera is always on the left of the center of the robot so it needs to slightly tilt to the the left (negative power)
        double cameraSidewayOffset = invertedCameraBearing - getLinearInterpolationOffsetBearing(range);

        // Normalize the bearing within a clamped range to get a -1 to 1 value for the power function
        double normalizedClampedBearing = Math.max(-CLAMP_BEARING, Math.min(CLAMP_BEARING, cameraSidewayOffset)) / CLAMP_BEARING;

        normalizedClampedBearing += (getXATPose() + CAMERA_POSITION_X) / (range + BASKET_Y_OFFSET) * SCALER_OFFSET_AT_TO_X_BASKET;

        return normalizedClampedBearing;
    }

    /**
     * Calculates the bearing from the robot's center to the AprilTag, compensating for camera offset.
     *
     * @return The corrected bearing in degrees.
     */
    public double getAutoAimPower(){
        // Calculate the auto-aim rotation power from the bearing (sign preserving)
        return pickCustomPowerFunc(getNormalizedClampedRobotCenterBasketBearing(), NUMBER_CUSTOM_POWER_FUNC_DRIVE_LOCKED);
    }

    public double getAutoAimPower2(){
        double theta = Math.max(-CLAMP_BEARING, Math.min(CLAMP_BEARING, getCameraBearing()));

        double y = getRangeToAprilTag();

        double xt2 = CAMERA_POSITION_X + (y + BASKET_Y_OFFSET) * Math.sin(theta);
        double yt2 = (y + BASKET_Y_OFFSET) * Math.cos(theta);


        return pickCustomPowerFunc(Math.atan2(xt2, yt2) / (2*Math.PI), NUMBER_CUSTOM_POWER_FUNC_DRIVE_LOCKED);
    }

    /**
     * Gets the cached averaged XftcPose from AprilTag detections.
     *
     * @return The averaged XftcPose in inches, or null if no data is available.
     */
    public Double getXATPose(){
        return cachedXATPose;
    }

    /**
     * Gets the cached averaged YftcPose from AprilTag detections.
     *
     * @return The averaged YftcPose in inches, or null if no data is available.
     */
    public Double getYATPose(){
        return cachedYATPose;
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
        return !robotPoseXSamples.isEmpty();
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
                x != null ? x : 0.0,
                y != null ? y : 0.0,
                yaw != null ? yaw : 0.0);
    }

    private void setAprilTagDetectionDecimation(int dec) {
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

        int optimalDecimation = (int) Math.round(4.0 - 3.0 * (rangeToAprilTag - 35) / 45.0);
        optimalDecimation = Math.max(1, Math.min(4, optimalDecimation));
        
        if (optimalDecimation != decimation) {
            setDecimation(optimalDecimation);
        }
    }

    public int getDecimation(){
        return decimation;
    }
    
    public void setDecimation(int newDecimation){
        decimation = newDecimation;
        setAprilTagDetectionDecimation(decimation);
    }

    @Override
    public void periodic() {
        printTelemetry(telemetry);
    }

    private void printTelemetry(final Telemetry telemetry) {
        if (builder == null) {
            telemetry.addLine("VisionPortal Builder is null");
            return;
        }
        if (aprilTagProcessor != null) aprilTagProcessorTelemetry();
        if (colorProcessor != null) colorProcessorTelemetry();
        if (aprilTagProcessor == null && colorProcessor == null) telemetry.addData("Vision", "No processors added");
    }

    private void aprilTagProcessorTelemetry() {
        telemetry.addData("Current AT Detections", getHasCurrentAprilTagDetections() ? "Yes" : "No");

        if (hasCachedPoseData()) {
            telemetry.addData("X Robot (CM)", "%.2f", getRobotPoseX());
            telemetry.addData("Y Robot (CM)", "%.2f", getRobotPoseY());
            telemetry.addData("Yaw (DEGREES)", "%.2f", getRobotPoseYaw());
            telemetry.addData("Camera Bearing (DEGREES)", "%.2f", getCameraBearing());
//            telemetry.addData("Robot Center Bearing", "%.2f", getRobotCenterBearing());
            telemetry.addData("Range (CM)", "%.2f", getRangeToAprilTag());
            telemetry.addData("X AT (CM)", "%.2f", getXATPose());
            telemetry.addData("Y AT (CM)", "%.2f", getYATPose());
//            telemetry.addData("Auto Aim Power")
//            telemetry.addData("Should Be 0", "%.2f", getNormalizedClampedRobotCenterBasketBearing());
        } else {
            telemetry.addData("AprilTag Pose Data", "No sample data");
        }

        telemetry.addData("Decimation", getDecimation());

        if (hasDetectedColorOrder) {
            telemetry.addData("Artifact Color Order", String.join(", ", cachedColorsOrder));
        }
    }

    private void colorProcessorTelemetry() {
        PredominantColorProcessor.Result result = colorProcessor.getAnalysis();
        telemetry.addData("Predominant Color", result != null ? result.closestSwatch : "No analysis");
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

}
