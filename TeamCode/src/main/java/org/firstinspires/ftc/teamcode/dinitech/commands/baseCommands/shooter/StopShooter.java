package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.shooter;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;

public class StopShooter extends WaitVelocityShooter {
    public StopShooter(ShooterSubsystem shooterSubsystem) {
        super(shooterSubsystem, 0);
    }
}
