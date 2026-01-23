package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.linearSpeedFromRange;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class InstantRangeVisionShooter extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;

    private double nullCounts = 0;

    /**
     * Creates a VisionShooter command that adjusts shooter speed based on AprilTag
     * range.
     *
     * @param shooterSubsystem The shooter subsystem
     * @param visionSubsystem  The vision subsystem
     */
    public InstantRangeVisionShooter(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void initialize(){
        shooterSubsystem.setUsageState(ShooterSubsystem.ShooterUsageState.VISION);
    }


    @Override
    public void execute() {
        Double range = visionSubsystem.getRangeToAprilTag();
        if (range == null){
            nullCounts += 1;
            return;
        }
        shooterSubsystem.setVelocity(linearSpeedFromRange(range));
    }

    @Override
    public boolean isFinished() {
        return nullCounts > 10;
    }

    @Override
    public void end(boolean interrupted) {
        if (nullCounts > 10){
            new SetVelocityShooter(shooterSubsystem, 1400).schedule();
        }
    }

}
