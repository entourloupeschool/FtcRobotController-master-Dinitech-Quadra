package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TELE_SHOOTER_SCALER;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

public class TeleShooter extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final GamepadWrapper operator;

    public TeleShooter(ShooterSubsystem shooterSubsystem, GamepadSubsystem gamepadSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.operator = gamepadSubsystem.operator;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.setUsageState(ShooterSubsystem.ShooterUsageState.TELE);
    }

    @Override
    public void execute(){
        double velocityIncrement = - operator.getRightY();
        if (Math.abs(velocityIncrement) > 0.05){
            shooterSubsystem.incrementVelocity(velocityIncrement * TELE_SHOOTER_SCALER);
        }
    }
}
