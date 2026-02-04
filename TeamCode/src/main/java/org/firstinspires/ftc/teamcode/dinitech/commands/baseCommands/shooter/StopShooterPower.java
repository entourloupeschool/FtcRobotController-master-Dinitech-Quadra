package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

public class StopShooterPower extends CommandBase {
    private ShooterSubsystem shooterSubsystem;

    public StopShooterPower(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
