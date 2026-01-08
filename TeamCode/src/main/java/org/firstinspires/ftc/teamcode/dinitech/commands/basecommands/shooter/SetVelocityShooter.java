package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_SHOOT_SPEED;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

public class SetVelocityShooter extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    protected final double ticksPerSecond;


    public SetVelocityShooter(ShooterSubsystem shooterSubsystem, double ticksPerSecond){
        this.shooterSubsystem = shooterSubsystem;
        this.ticksPerSecond = ticksPerSecond;
    }

    @Override
    public void initialize(){
        shooterSubsystem.setVelocity(ticksPerSecond);
    }


    @Override
    public boolean isFinished() {
        return shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN);
    }


}
