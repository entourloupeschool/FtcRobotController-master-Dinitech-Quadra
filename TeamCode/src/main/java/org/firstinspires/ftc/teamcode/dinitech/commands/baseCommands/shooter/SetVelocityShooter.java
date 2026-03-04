package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter;

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
        return true;
    }


}
