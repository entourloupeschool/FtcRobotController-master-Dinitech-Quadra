package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * A command that runs continuously to keep the AprilTag detection data up-to-date.
 * <p>
 * This command is intended to be set as the default command for the {@link VisionSubsystem}.
 * In each execution cycle, it performs two key actions:
 * <ol>
 *     <li>Calls {@code optimizeDecimation()} to dynamically adjust the AprilTag processor's
 *     performance based on the current distance to the tag.</li>
 *     <li>Calls {@code updateAprilTagDetections()} to process the latest camera frame and
 *     update the robot's pose estimation.</li>
 * </ol>
 * Because this command never finishes, it ensures that the vision subsystem is always
 * providing the most current data possible.
 */
public class ContinuousUpdatesAprilTagsDetections extends CommandBase {
    private final VisionSubsystem visionSubsystem;

    /**
     * Creates a new ContinuousUpdateAprilTagsDetections command.
     *
     * @param visionSubsystem The vision subsystem to continuously update.
     */
    public ContinuousUpdatesAprilTagsDetections(VisionSubsystem visionSubsystem){
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize(){
        visionSubsystem.setUsageState(VisionSubsystem.VisionUsageState.CONTINUOUS);
    }

    /**
     * Optimizes and updates the AprilTag detections in each execution cycle.
     */
    @Override
    public void execute(){
//        visionSubsystem.optimizeDecimation();
        visionSubsystem.updateAprilTagDetections();
    }
}
