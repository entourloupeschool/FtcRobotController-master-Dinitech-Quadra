package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_RANGE_TO_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_SHOOT_SPEED;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MIN_RANGE_TO_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN_VISION_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.linearSpeedFromRange;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class VisionShooter extends CommandBase {
    private static final int NUM_SPEED_LEVELS = 20;

    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final boolean continuous;

    private int currentSpeedLevel = -1; // Track current speed level to avoid redundant calls
    private double targetShooterSpeed = MAX_SHOOT_SPEED;

    /**
     * Creates a VisionShooter command that adjusts shooter speed based on AprilTag
     * range.
     *
     * @param shooterSubsystem The shooter subsystem
     * @param visionSubsystem  The vision subsystem
     * @param continuous       If true, continuously updates speed in execute(). If
     *                         false, sets speed once in initialize() and finishes
     *                         when target speed is reached.
     */
    public VisionShooter(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, boolean continuous) {
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.continuous = continuous;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        currentSpeedLevel = -1; // Reset speed level tracking

        if (!continuous) {
            // For instant mode, set speed once in initialize
            updateShooterSpeed();
        } else {
            shooterSubsystem.setVisionShooting(true);
        }
    }

    @Override
    public void execute() {
        if (continuous) {
            // For continuous mode, update speed every cycle
            updateShooterSpeed();
        }
    }

    @Override
    public boolean isFinished() {
        // Continuous mode never finishes, instant mode finishes when target speed is
        // reached
        return !continuous && shooterSubsystem.isSpeedAround(targetShooterSpeed, SPEED_MARGIN_VISION_SHOOT);
    }

    /**
     * Updates the shooter speed based on the current AprilTag range.
     * Uses discrete speed levels to avoid redundant setVelocity() calls.
     */
    private void updateShooterSpeed() {
        boolean hasData = continuous ? visionSubsystem.getHasCurrentAprilTagDetections()
                : visionSubsystem.hasCachedPoseData();

        if (hasData) {
//            // Calculate which speed level (0-9) based on range
//            // Clamp range to [0, MAX_RANGE_TO_SHOOT] to ensure valid speed level
//            double clampedRange = Math.max(MIN_RANGE_TO_SHOOT, Math.min(visionSubsystem.getRangeToAprilTag(), MAX_RANGE_TO_SHOOT)) - MIN_RANGE_TO_SHOOT;
//            int speedLevel = (int) ((clampedRange / (MAX_RANGE_TO_SHOOT - MIN_RANGE_TO_SHOOT)) * (NUM_SPEED_LEVELS - 1));
//
//            // Only update velocity if speed level has changed
//            if (speedLevel != currentSpeedLevel) {
//                currentSpeedLevel = speedLevel;
//
//                // Calculate actual speed for this level
//                targetShooterSpeed = MAX_SHOOT_SPEED * (double) speedLevel / (NUM_SPEED_LEVELS - 1);
//                shooterSubsystem.setVelocity(targetShooterSpeed);
//            }

            shooterSubsystem.setVelocity(linearSpeedFromRange(visionSubsystem.getRangeToAprilTag()));
        }
    }
}
