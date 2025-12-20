package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MAX_SHOOT_SPEED;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

public class MaxSpeedShooter extends SetVelocityShooter {
    public MaxSpeedShooter(ShooterSubsystem shooterSubsystem){
        super(shooterSubsystem, MAX_SHOOT_SPEED);
    }
}
