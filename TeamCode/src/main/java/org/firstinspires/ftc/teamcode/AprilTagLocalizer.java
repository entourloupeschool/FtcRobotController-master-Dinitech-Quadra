package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Config
public class AprilTagLocalizer implements Localizer {
    private final Localizer baseLocalizer;
    private final VisionSubsystem visionSubsystem;

    /**
     * Constructor for AprilTagLocalizer using VisionSubsystem
     * @param baseLocalizer The base localizer (e.g., PinpointLocalizer)
     * @param visionSubsystem The vision subsystem that provides AprilTag data
     */
    public AprilTagLocalizer(Localizer baseLocalizer, VisionSubsystem visionSubsystem) {
        this.baseLocalizer = baseLocalizer;
        this.visionSubsystem = visionSubsystem;
    }

    private Pose2d offset = new Pose2d(0.0, 0.0, 0.0);

    // nicer logging
    private final Map<String, Object> cachedValues = new HashMap<>();

    public Pose2d getBasePose() {
        return baseLocalizer.getPose();
    }

    public void setBasePose(Pose2d pose) {
        baseLocalizer.setPose(pose);
    }

    @Override
    public Pose2d getPose() {
        return getBasePose().times(offset);
    }

    @Override
    public void setPose(Pose2d pose) {
        setBasePose(pose);
        offset = new Pose2d(0.0, 0.0, 0.0);
    }

    public void setPoseOffset(Pose2d pose) {
        offset = pose.times(getBasePose().inverse());
    }

    private void log(String ch, Object value) {
        if (!cachedValues.containsKey(ch)) {
            cachedValues.put(ch, value);
            FlightRecorder.write(ch, value);
            return;
        }

        if (!cachedValues.get(ch).equals(value)) {
            cachedValues.put(ch, value);
            FlightRecorder.write(ch, value);
        }
    }

    /**
     * Updates the Localizer's pose estimate using VisionSubsystem data.
     * @return the Localizer's current velocity estimate
     */
    @Override
    public PoseVelocity2d update() {
        PoseVelocity2d vel = baseLocalizer.update();
        log("AprilTagLocalizer/basePose", getBasePose().toString());
        log("AprilTagLocalizer/offset", offset.toString());
        log("AprilTagLocalizer/pose", getPose().toString());
        log("AprilTagLocalizer/correctedThisLoop", false);

        // Skip AprilTag correction if moving too fast (optional optimization)
        if (vel.linearVel.norm() > 1.0 || Math.toDegrees(vel.angVel) > 1.0) {
            return vel;
        }

        // Use VisionSubsystem data if available
        if (visionSubsystem.getHasCurrentAprilTagDetections()) {

                // Get pose from vision data
                Pose2d aprilTagPose = visionSubsystem.getLatestRobotPoseEstimationFromAT();

                // Only apply correction if the pose seems reasonable
                // (within some distance of current pose to avoid bad detections)
                double distanceFromCurrent = aprilTagPose.position.minus(getPose().position).norm();
                
                if (distanceFromCurrent < 24.0) { // Within 24 inches - adjust as needed
                    setPoseOffset(aprilTagPose);
                    log("AprilTagLocalizer/pose", getPose().toString());
                    log("AprilTagLocalizer/correctedThisLoop", true);
                    log("AprilTagLocalizer/correctionDistance", distanceFromCurrent);
                } else {
                    log("AprilTagLocalizer/rejectedCorrection", distanceFromCurrent);
                }

        }

        return vel;
    }
}
