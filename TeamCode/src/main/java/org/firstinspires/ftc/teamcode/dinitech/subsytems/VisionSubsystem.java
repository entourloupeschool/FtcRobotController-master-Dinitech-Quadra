package org.firstinspires.ftc.teamcode.dinitech.subsytems;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CLAMP_BEARING;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CORRECTION_BASKET_OFFSET;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CX;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FX;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FY;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA1_NAME;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_ORIENTATION_PITCH;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_ORIENTATION_ROLL;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_ORIENTATION_YAW;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_POSITION_X;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_POSITION_Y;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.CAMERA_POSITION_Z;
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
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.FTCCoordinates;
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
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision.ContinuousUpdatesAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.other.Globals;
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
    public final TelemetryManager telemetryM;
    public VisionPortal visionPortal = null;
    public AprilTagProcessor aprilTagProcessor;
    public PredominantColorProcessor colorProcessor;

    private final Globals.RunningAverage robotPoseXCMSamples = new Globals.RunningAverage(NUMBER_AT_SAMPLES);
    private final Globals.RunningAverage robotPoseYCMSamples = new Globals.RunningAverage(NUMBER_AT_SAMPLES);
    private final Globals.RunningAverage robotPoseYDEGREESSamples = new Globals.RunningAverage(NUMBER_AT_SAMPLES);
    private final Globals.RunningAverage rangeToAprilTagCMSamples = new Globals.RunningAverage(NUMBER_AT_SAMPLES);
    private final Globals.RunningAverage cameraBearingDEGREESSamples = new Globals.RunningAverage(NUMBER_AT_SAMPLES);
    private final Globals.RunningAverage confidenceAprilTagSamples = new Globals.RunningAverage(NUMBER_AT_SAMPLES);
    private final Globals.RunningAverage aTPoseXCMSamples = new Globals.RunningAverage(NUMBER_AT_SAMPLES);
    private final Globals.RunningAverage aTPoseYCMSamples = new Globals.RunningAverage(NUMBER_AT_SAMPLES);

    private Double cachedRobotPoseXCM, cachedRobotPoseYCM, cachedRobotPoseYawDEGREES, cachedRangeToAprilTagCM, cachedCameraBearingDEGREES, cachedConfidence, cachedXATPoseCM, cachedYATPoseCM;

    private boolean hasCurrentATDetections = false;
    private int cachedColorsOrder = -1;
    private boolean hasDetectedColorOrder = false;
    private double decimation = 1;
    private int lastATPositionDetection = 20;

    public enum VisionUsageState { OPTIMIZED, CONTINUOUS, AT, COLOR, NONE }
    private VisionUsageState usageState;

    public void setUsageState(VisionUsageState state) { this.usageState = state; }
    public VisionUsageState getUsageState() { return usageState; }

    public VisionSubsystem(HardwareMap hardwareMap, final TelemetryManager telemetryM) {
        this.aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setDrawCubeProjection(true)
                .setCameraPose(
                        new Position(DistanceUnit.CM, CAMERA_POSITION_X, CAMERA_POSITION_Y, CAMERA_POSITION_Z, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, CAMERA_ORIENTATION_YAW, CAMERA_ORIENTATION_PITCH, CAMERA_ORIENTATION_ROLL, 0))
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

        setUsageState(VisionUsageState.NONE);

        this.telemetryM = telemetryM;
    }

    public void updateAprilTagDetections() {
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
                        cachedColorsOrder = detection.id;
                        hasDetectedColorOrder = true;
                    }
                } else {
                    setHasCurrentAprilTagDetections(false);
                }
            }
    }

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

    public Double getRobotPoseX() { return cachedRobotPoseXCM; }
    public Double getRobotPoseY() { return cachedRobotPoseYCM; }
    public Double getRobotPoseYaw() { return cachedRobotPoseYawDEGREES; }
    public Double getRangeToAprilTag() { return cachedRangeToAprilTagCM; }
    public Double getCameraBearing() { return cachedCameraBearingDEGREES; }

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
        double cameraBearing = getCameraBearing() != null ? getCameraBearing() : 0;
        double xRobot = getRobotPoseX() != null ? getRobotPoseX() : 0;
        double yRobot = getRobotPoseY() != null ? getRobotPoseY() : 0;
        double rangeToAT = getRangeToAprilTag() != null ? getRangeToAprilTag() : 50;
        double robotCenterBearing = getRobotCenterToAprilTag(cameraBearing, rangeToAT);
        double normalizedCorrectionWithRange = getNormalizedCorrectionWithRange(getSignedDistanceToATLine(xRobot, yRobot, lastATPositionDetection), rangeToAT);
        return pickCustomPowerFunc(Math.max(-CLAMP_BEARING, Math.min(CLAMP_BEARING, robotCenterBearing + normalizedCorrectionWithRange)) / CLAMP_BEARING, NUMBER_CUSTOM_POWER_FUNC_DRIVE_LOCKED);
    }

    public Double getXATPose(){ return cachedXATPoseCM; }
    public Double getYATPose(){ return cachedYATPoseCM; }

    public boolean getHasCurrentAprilTagDetections() { return hasCurrentATDetections; }
    public void setHasCurrentAprilTagDetections(boolean val) { hasCurrentATDetections = val; }
    public boolean hasCachedPoseData() { return !robotPoseXCMSamples.isEmpty(); }

    public Pose getLatestRobotPoseEstimationFromAT() {
        Double x = getRobotPoseX();
        Double y = getRobotPoseY();
        Double yaw = getRobotPoseYaw();
        if (x == null || y == null || yaw == null) return new Pose(0.0, 0.0, 0.0);

        return new Pose(cmToInch(x), cmToInch(y), Math.toRadians(yaw), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        // Use Pose2D from FTC SDK and convert it to Pedro Pose using FTC coordinate system
//        return PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, yaw), PedroCoordinates.INSTANCE);
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
        telemetryM.addData("vision usage state", getUsageState());
    }

    private void aprilTagProcessorTelemetryManager() {
//        telemetryM.addData("Current AT Detections", getHasCurrentAprilTagDetections() ? "Yes" : "No");
        if (hasCachedPoseData()) {
            Pose lastDetectedPose = getLatestRobotPoseEstimationFromAT();
            telemetryM.addLine("last detected ATpose inPedroCoord");
            telemetryM.addData("X", lastDetectedPose.getX());
            telemetryM.addData("Y",  lastDetectedPose.getY());
            telemetryM.addData("heading",  lastDetectedPose.getHeading());
        }
        telemetryM.addData("hasDetectedMotif", hasDetectedColorOrder);
    }

    public int getColorsOrder() { return cachedColorsOrder; }
    public boolean hasColorOrder() { return hasDetectedColorOrder; }
    public void resetColorOrder() {
        cachedColorsOrder = -1;
        hasDetectedColorOrder = false;
    }

    public void setAprilTagProcessorEnabled(boolean val){ if (visionPortal != null) visionPortal.setProcessorEnabled(aprilTagProcessor, val); }
    public boolean getAprilTagProcessorEnabled(){ return visionPortal != null && visionPortal.getProcessorEnabled(aprilTagProcessor); }
    public VisionPortal getVisionPortal(){ return visionPortal; }
    public void turnCameraOff(){ if (visionPortal != null) visionPortal.stopStreaming(); }
    public void turnCameraOn(){ if (visionPortal != null) visionPortal.resumeStreaming(); }
}
