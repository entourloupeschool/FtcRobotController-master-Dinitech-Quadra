package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

public class SetVelocityShooterRequire extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    protected final double ticksPerSecond;


    public SetVelocityShooterRequire(ShooterSubsystem shooterSubsystem, double ticksPerSecond){
        this.shooterSubsystem = shooterSubsystem;
        this.ticksPerSecond = ticksPerSecond;
        addRequirements(shooterSubsystem);
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
