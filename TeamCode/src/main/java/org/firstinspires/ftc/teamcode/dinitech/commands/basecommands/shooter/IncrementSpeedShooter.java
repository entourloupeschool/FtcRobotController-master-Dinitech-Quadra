package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TELE_SHOOTER_SCALER;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

public class IncrementSpeedShooter extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;

    private final double velocityIncrement;


    public IncrementSpeedShooter(ShooterSubsystem shooterSubsystem, double velocityIncrement){
        this.shooterSubsystem = shooterSubsystem;
        this.velocityIncrement = velocityIncrement;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.incrementVelocity(velocityIncrement * TELE_SHOOTER_SCALER);
    }


    @Override
    public boolean isFinished()
    {
        return true;
    }


}
