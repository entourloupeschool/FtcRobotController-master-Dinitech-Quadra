package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_HIGH_SPEED_TRIEUR;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class MoulinHighSpeedIntel extends SequentialCommandGroup {

    public MoulinHighSpeedIntel(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
            new MoulinNextShootIntel(trieurSubsystem, shooterSubsystem),
            new ParallelRaceGroup(
                    new WaitUntilCommand(shooterSubsystem::isCurrentOverflow),
                    new WaitCommand(WAIT_HIGH_SPEED_TRIEUR)),
            new MoulinNextShootIntel(trieurSubsystem, shooterSubsystem),
            new ParallelRaceGroup(
                    new WaitUntilCommand(shooterSubsystem::isCurrentOverflow),
                    new WaitCommand(WAIT_HIGH_SPEED_TRIEUR)),
            new MoulinNextShootIntel(trieurSubsystem, shooterSubsystem)
        );
    }
}
