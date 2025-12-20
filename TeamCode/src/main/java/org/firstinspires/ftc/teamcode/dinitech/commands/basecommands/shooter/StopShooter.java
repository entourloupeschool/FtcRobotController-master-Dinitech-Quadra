package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

public class StopShooter extends SetVelocityShooter {
    public StopShooter(ShooterSubsystem shooterSubsystem) {
        super(shooterSubsystem, 0);
    }
}
