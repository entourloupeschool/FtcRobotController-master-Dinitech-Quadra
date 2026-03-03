package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class MoulinHighSpeedIntel extends SequentialCommandGroup {

    public MoulinHighSpeedIntel(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem, CommandBase waitCommand) {
        addCommands(
            new MoulinNextShootIntel(trieurSubsystem, shooterSubsystem),
            waitCommand,
            new MoulinNextShootIntel(trieurSubsystem, shooterSubsystem),
            waitCommand,
            new MoulinNextShootIntel(trieurSubsystem, shooterSubsystem)
        );
    }
}
