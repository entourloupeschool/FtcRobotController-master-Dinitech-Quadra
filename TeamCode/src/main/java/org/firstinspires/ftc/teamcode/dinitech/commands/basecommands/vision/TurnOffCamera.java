package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * An instant command that triggers a single update of the AprilTag detections.
 * <p>
 * When scheduled, this command calls the {@code updateAprilTagDetections()} method on the
 * {@link VisionSubsystem}. This processes the latest camera frame to find AprilTags,
 * update their positions, and add new data to the running average filters for pose estimation.
 * The command finishes immediately after initiating the update.
 */
public class TurnOffCamera extends CommandBase {
    private final VisionSubsystem visionSubsystem;

    /**
     * Creates a new UpdateAprilTagsDetections command.
     *
     * @param visionSubsystem The vision subsystem to update.
     */
    public TurnOffCamera(VisionSubsystem visionSubsystem){
        this.visionSubsystem = visionSubsystem;
    }

    /**
     * Triggers the AprilTag detection update.
     */
    @Override
    public void initialize(){
//        visionSubsystem.setCameraOn(false);
    }

    /**
     * This command is always finished immediately.
     *
     * @return Always returns true.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
