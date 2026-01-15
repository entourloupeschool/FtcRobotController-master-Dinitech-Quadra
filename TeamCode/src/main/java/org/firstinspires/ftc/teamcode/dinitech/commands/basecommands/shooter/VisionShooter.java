package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_RANGE_SHOOTER_SPEED;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN_VISION_SHOOT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.linearSpeedFromRange;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdateAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class VisionShooter extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final boolean continuous;

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
    public void initialize(){
        if (continuous){
            shooterSubsystem.setUsageState(ShooterSubsystem.ShooterUsageState.VISION);
        }
    }


    @Override
    public void execute() {
        shooterSubsystem.setVelocity(linearSpeedFromRange(visionSubsystem.getRangeToAprilTag()));
    }

    @Override
    public boolean isFinished() {
        // Continuous mode never finishes, instant mode finishes when target speed is
        // reached
        return !continuous && shooterSubsystem.isSpeedAround(shooterSubsystem.getTargetSpeed(), SPEED_MARGIN_VISION_SHOOT);
    }
}
