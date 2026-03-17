package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_HIGH_SPEED_TRIEUR;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.groups.WaitShoot;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class MoulinHighSpeedIntel extends SequentialCommandGroup {

    public MoulinHighSpeedIntel(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
            new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
            new WaitShoot(shooterSubsystem),
            new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem),
            new WaitShoot(shooterSubsystem),
            new MoulinNextArtefactShootWaitVelocity(trieurSubsystem, shooterSubsystem)
        );
    }
}
