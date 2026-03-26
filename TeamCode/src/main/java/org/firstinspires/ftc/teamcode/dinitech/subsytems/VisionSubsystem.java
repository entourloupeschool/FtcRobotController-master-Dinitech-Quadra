package org.firstinspires.ftc.teamcode.dinitech.subsytems;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.cmToInch;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.inchToCm;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.pickCustomPowerFunc;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.RunningAverage;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem.MAX_RANGE_TO_SHOOT_CM;
import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem.MIN_RANGE_TO_SHOOT_CM;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

/**
 * A command-based subsystem that manages the robot's vision capabilities.
 */
public class VisionSubsystem extends SubsystemBase {
    public static final String CAMERA1_NAME = "CamColor";
    public static final double FX = 516.3798424;//0.0;// 516.3798424;//1;
    // https://github.com/jdhs-ftc/2025/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/atag/AprilTagLocalizer.kt
    public static final double FY = 515.8231389;//0.0;// 515.8231389; //1;
    public static final double CX = 328.1776587;// 0.0;// 328.1776587; //1;
    public static final double CY = 237.3745503;//0.0;// 237.3745503; //1;
    public static final double CAMERA_POSITION_X = -9.8;
    public static final double CAMERA_POSITION_Y = 2.8;
    public static final double CAMERA_POSITION_Z = 43.0;
    public static final double CAMERA_ORIENTATION_YAW = 0;
    public static final double CAMERA_ORIENTATION_PITCH = -90; // https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_localization/apriltag-localization.html
    public static final double CAMERA_ORIENTATION_ROLL = 0;
    // Choose a camera resolution. Not all cameras support all resolutions.
    public static final int CAMERA_WIDTH = 1280;// 1280; // 640;
    public static final int CAMERA_HEIGHT = 800;// 800; // 480;
    public static final Size CAMERA_RESOLUTION = new Size(CAMERA_WIDTH, CAMERA_HEIGHT); // new Size(640, 480);

    // Set the stream format; MJPEG uses less bandwidth than default YUY2.
    public static final VisionPortal.StreamFormat STREAM_FORMAT = VisionPortal.StreamFormat.MJPEG; // Or YUY2

    public static final double CLAMP_BEARING = 73;
    public static final double MIN_RANGE_VISION = MIN_RANGE_TO_SHOOT_CM; //CM
    public static final double MAX_RANGE_VISION = MAX_RANGE_TO_SHOOT_CM; //CM

    public static final double DIFF_RANGE_VISION = MAX_RANGE_VISION - MIN_RANGE_VISION; // = 248;

    public static final double OFFSET_BEARING_AT_MIN_RANGE = -4.45; // DEGREES
    public static final double OFFSET_BEARING_AT_MAX_RANGE = -1.8; //DEGREES
    public static final double DIFF_OFFSET_BEARING_AT = OFFSET_BEARING_AT_MAX_RANGE - OFFSET_BEARING_AT_MIN_RANGE; // = -2.65
    public static final double CORRECTION_BASKET_OFFSET = 0.21;
    public static final double BASKET_Y_OFFSET = 8;
    public static final int NUMBER_AT_SAMPLES = 3;
    public static final int NUMBER_CUSTOM_POWER_FUNC_DRIVE_LOCKED = 4;
    public static final double MIN_LINEAR = 0.005;
    public static final double CUSTOM_POWER_LOCKED = 0.05;

    public static final double DIFF_A_BEARING = DIFF_OFFSET_BEARING_AT / DIFF_RANGE_VISION; // = -0.01068548387

    public static final double aAT_LINE = 1.4;
    public static final double bAT_LINE = 59;

    /**
     * Calculates linear interpolation bearing offset using linear interpolation based on range.
     * This helps correct for parallax error due to camera placement.
     * @param range The range to the target.
     * @return The calculated bearing offset.
     */
    public static double getLinearInterpolationOffsetBearing(double range) {
        return DIFF_A_BEARING * range -5.4865;
    }


    public final TelemetryManager telemetryM;
    public VisionPortal visionPortal = null;
    public AprilTagProcessor aprilTagProcessor;
    public PredominantColorProcessor colorProcessor;

    private final RunningAverage robotPoseXSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage robotPoseYSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage robotPoseYawSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage rangeToATSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage cameraBearingSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage confidenceATSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage aTPoseXSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    private final RunningAverage aTPoseYSamples = new RunningAverage(NUMBER_AT_SAMPLES);
    
    private Double cachedRobotPoseX, cachedRobotPoseY, cachedRobotPoseYaw, cachedRangeToAT, cachedCameraBearing, cachedConfidence, cachedATPoseX, cachedATPoseY;

    private boolean hasCurrentATDetections = false;
    private int cachedMotif = -1;
    private double decimation = 1;
    private int lastATGoalID = 20;

    public enum VisionUsageState { MOTIF, OPTIMIZED, CONTINUOUS, AT, COLOR, NONE }
    private VisionUsageState usageState;

    public void setUsageState(VisionUsageState state) { this.usageState = state; }
    public VisionUsageState getUsageState() { return usageState; }

    public VisionSubsystem(HardwareMap hardwareMap, final TelemetryManager telemetryM) {
        this.aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setDrawCubeProjection(true)
                .setCameraPose(
                        new Position(DistanceUnit.INCH, cmToInch(CAMERA_POSITION_X), cmToInch(CAMERA_POSITION_Y), cmToInch(CAMERA_POSITION_Z), 0),
                        new YawPitchRollAngles(AngleUnit.RADIANS, Math.toRadians(CAMERA_ORIENTATION_YAW), Math.toRadians(CAMERA_ORIENTATION_PITCH), Math.toRadians(CAMERA_ORIENTATION_ROLL), 0))
                .setLensIntrinsics(FX, FY, CX, CY)
//                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        this.visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA1_NAME))
                .setStreamFormat(STREAM_FORMAT)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setAutoStopLiveView(true)
                .enableLiveView(false)
                .build();

        setUsageState(VisionUsageState.NONE);

        setCachedMotif(-1);

        this.telemetryM = telemetryM;
    }

    public void updateAprilTagDetections() {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            hasCurrentATDetections = !detections.isEmpty();
            if (hasCurrentATDetections) {
                AprilTagDetection detection = detections.get(0);
                if (detection.metadata != null) {
                    if (detection.id == 20 || detection.id == 24) {
                        lastATGoalID = detection.id;
                        robotPoseXSamples.add(detection.robotPose.getPosition().x);
                        robotPoseYSamples.add(detection.robotPose.getPosition().y);
                        robotPoseYawSamples.add(detection.robotPose.getOrientation().getYaw());
                        cameraBearingSamples.add(detection.ftcPose.bearing);
                        rangeToATSamples.add(detection.ftcPose.range);
                        confidenceATSamples.add(detection.decisionMargin);
                        aTPoseXSamples.add(detection.ftcPose.x);
                        aTPoseYSamples.add(detection.ftcPose.y);
                        updateCachedAverages();
                    } else if (!hasMotif()) {
                        cachedMotif = detection.id;
                    }
                } else {
                    setHasCurrentAprilTagDetections(false);
                }
            }
    }
    
    public void updateMotifDetection(){
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        hasCurrentATDetections = !detections.isEmpty();
        if (hasCurrentATDetections) {
            for (AprilTagDetection detection : detections){
                if (detection.metadata != null) {
                    int detectionId = detection.id;
                    if (detectionId == 21 || detectionId == 22 || detectionId == 23){
                        cachedMotif = detectionId;
                        break;
                    }
                }
            }
        }
    }

    public void updateCachedAverages() {
        cachedRobotPoseX = robotPoseXSamples.getAverage();
        cachedRobotPoseY = robotPoseYSamples.getAverage();
        cachedRobotPoseYaw = robotPoseYawSamples.getAverage();
        cachedRangeToAT = rangeToATSamples.getAverage();
        cachedCameraBearing = cameraBearingSamples.getAverage();
        cachedConfidence = confidenceATSamples.getAverage();
        cachedATPoseX = aTPoseXSamples.getAverage();
        cachedATPoseY = aTPoseYSamples.getAverage();
    }

    public Double getRobotPoseX() { return cachedRobotPoseX; }
    public Double getRobotPoseY() { return cachedRobotPoseY; }
    public Double getRobotPoseYaw() { return cachedRobotPoseYaw; }
    public Double getRangeToAprilTag() { return cachedRangeToAT; }
    public Double getCameraBearing() { return cachedCameraBearing; }

    public static double getRobotCenterToAprilTag(double cameraBearing, double rangeToAprilTag){
        return -cameraBearing + getLinearInterpolationOffsetBearing(rangeToAprilTag);
    }

    public static double getSignedDistanceToATLine(double xRobot, double yRobot, int tagID) {
        if (tagID == 20) return (yRobot - aAT_LINE * xRobot - bAT_LINE) / Math.sqrt(aAT_LINE * aAT_LINE + 1);
        else if (tagID == 24) return (yRobot + aAT_LINE * xRobot + bAT_LINE) / Math.sqrt(aAT_LINE * aAT_LINE + 1);
        else return 0;
    }

    public static double getNormalizedCorrectionWithRange(double signedDistanceToATLine, double rangeToAT) {
        return signedDistanceToATLine / (CORRECTION_BASKET_OFFSET * rangeToAT);
    }

    public double getAutoAimPower(){
        double cameraBearing = getCameraBearing() != null ? Math.toRadians(getCameraBearing()) : 0;
        double xRobot = getRobotPoseX() != null ? inchToCm(getRobotPoseX()) : 0;
        double yRobot = getRobotPoseY() != null ? inchToCm(getRobotPoseY()) : 0;
        double rangeToAT = getRangeToAprilTag() != null ? inchToCm(getRangeToAprilTag()) : 50;
        double robotCenterBearing = getRobotCenterToAprilTag(cameraBearing, rangeToAT);
        double normalizedCorrectionWithRange = getNormalizedCorrectionWithRange(getSignedDistanceToATLine(xRobot, yRobot, lastATGoalID), rangeToAT);
        return pickCustomPowerFunc(Math.max(-CLAMP_BEARING, Math.min(CLAMP_BEARING, robotCenterBearing + normalizedCorrectionWithRange)) / CLAMP_BEARING, NUMBER_CUSTOM_POWER_FUNC_DRIVE_LOCKED);
    }

    public Double getXATPose(){ return cachedATPoseX; }
    public Double getYATPose(){ return cachedATPoseY; }

    public boolean getHasCurrentAprilTagDetections() { return hasCurrentATDetections; }
    public void setHasCurrentAprilTagDetections(boolean val) { hasCurrentATDetections = val; }
    public boolean hasCachedPoseData() { return !robotPoseXSamples.isEmpty(); }

    public Pose getLatestRobotPoseEstimationFromAT() {
        return PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH, getRobotPoseX(), getRobotPoseY(), AngleUnit.RADIANS, getRobotPoseYaw()), InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

//        return new Pose(x, y, yaw);

//        return new Pose(cmToInch(x), cmToInch(y), Math.toRadians(yaw), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        // Use Pose2D from FTC SDK and convert it to Pedro Pose using FTC coordinate system
    }

    public void optimizeDecimation() {
        if (aprilTagProcessor == null) return;
        Double rangeToAprilTag = getRangeToAprilTag();
        if (rangeToAprilTag == null) return;
        float optimalDecimation = (float) Math.round(4.0 - 3.0 * (rangeToAprilTag - 35) / 45.0);
        setDecimation(Math.max(1, Math.min(4, optimalDecimation)));
    }

    public double getDecimation(){ return decimation; }
    public void setDecimation(float val){
        decimation = val;
        if (aprilTagProcessor != null) aprilTagProcessor.setDecimation((float) decimation);
    }

    @Override
    public void periodic() {
        aprilTagProcessorTelemetryManager();
//        telemetryM.addData("vision usage state", getUsageState());
    }

    private void aprilTagProcessorTelemetryManager() {
//        telemetryM.addData("Current AT Detections", getHasCurrentAprilTagDetections() ? "Yes" : "No");
//        if (hasCachedPoseData()) {
//            Pose lastDetectedPose = getLatestRobotPoseEstimationFromAT();
//            telemetryM.addLine("last detected ATpose inPedroCoord");
//            telemetryM.addData("X", lastDetectedPose.getX());
//            telemetryM.addData("Y",  lastDetectedPose.getY());
//            telemetryM.addData("heading",  lastDetectedPose.getHeading());
//            telemetryM.addData("yawRaw", getRobotPoseYaw());
//        }

        telemetryM.addData("hasDetectedMotif", hasMotif());
    }

    public int getCachedMotif() { return cachedMotif; }
    public void setCachedMotif(int val){
        cachedMotif = val;
    }
    public boolean hasMotif() { return getCachedMotif() != -1; }

    public void setAprilTagProcessorEnabled(boolean val){ if (visionPortal != null) visionPortal.setProcessorEnabled(aprilTagProcessor, val); }
    public boolean getAprilTagProcessorEnabled(){ return visionPortal != null && visionPortal.getProcessorEnabled(aprilTagProcessor); }
    public VisionPortal getVisionPortal(){ return visionPortal; }
    public void turnCameraOff(){ if (visionPortal != null) visionPortal.stopStreaming(); }
    public void turnCameraOn(){ if (visionPortal != null) visionPortal.resumeStreaming(); }
}
