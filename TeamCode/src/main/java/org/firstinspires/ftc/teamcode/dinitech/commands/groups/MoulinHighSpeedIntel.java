package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.groups.WaitShoot;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class MoulinHighSpeedIntel extends SequentialCommandGroup {

    public MoulinHighSpeedIntel(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
            new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
            new WaitShoot(shooterSubsystem, trieurSubsystem),
            new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
            new WaitShoot(shooterSubsystem, trieurSubsystem),
            new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem)
        );
    }
}
