package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
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
public class OptimizedUpdatesAprilTagsDetections extends CommandBase {
    private final VisionSubsystem visionSubsystem;
    private final DrivePedroSubsystem drivePedroSubsystem;
    private final TrieurSubsystem trieurSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    /**
     * Creates a new OnlyUpdateAprilTagsDetections command.
     *
     * @param visionSubsystem The vision subsystem to continuously update.
*    * @param driveSubsystem The drive subsystem to continuously update.
     * @param shooterSubsystem The shooter subsystem to continuously update.
     */
    public OptimizedUpdatesAprilTagsDetections(VisionSubsystem visionSubsystem, DrivePedroSubsystem drivePedroSubsystem, TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem){
        this.visionSubsystem = visionSubsystem;
        this.drivePedroSubsystem = drivePedroSubsystem;
        this.trieurSubsystem = trieurSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize(){
        visionSubsystem.setUsageState(VisionSubsystem.VisionUsageState.OPTIMIZED);
    }


    /**
     * Optimizes and updates the AprilTag detections in each execution cycle.
     */
    @Override
    public void execute(){
        if (trieurSubsystem.getIsFull() || shooterSubsystem.getUsageState() == ShooterSubsystem.ShooterUsageState.VISION
                || drivePedroSubsystem.getDriveReference() == DrivePedroSubsystem.DriveReference.FC || drivePedroSubsystem.getDriveUsage() == DrivePedroSubsystem.DriveUsage.AIM_LOCKED) {
            if (!visionSubsystem.getAprilTagProcessorEnabled()) visionSubsystem.setAprilTagProcessorEnabled(true);

            visionSubsystem.optimizeDecimation();
            visionSubsystem.updateAprilTagDetections();
        } else {
            if (visionSubsystem.getAprilTagProcessorEnabled()){
                visionSubsystem.setAprilTagProcessorEnabled(false);
                visionSubsystem.setHasCurrentAprilTagDetections(false);
            }
        }
    }

    /**
     * This command should run indefinitely as a default command.
     *
     * @return Always returns false.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
