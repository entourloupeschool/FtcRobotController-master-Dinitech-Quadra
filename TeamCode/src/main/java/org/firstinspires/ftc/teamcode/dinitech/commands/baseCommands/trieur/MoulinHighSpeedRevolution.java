package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class MoulinHighSpeedRevolution extends SequentialCommandGroup {

    public MoulinHighSpeedRevolution(TrieurSubsystem trieurSubsystem, CommandBase waitCommand, CommandBase waitUntilCommand) {
        addCommands(
            waitUntilCommand,
            new MoulinNextShoot(trieurSubsystem),
            waitCommand,
            waitUntilCommand,
            new MoulinNextShoot(trieurSubsystem),
            waitCommand,
            waitUntilCommand,
            new MoulinNextShoot(trieurSubsystem));
    }

    public MoulinHighSpeedRevolution(TrieurSubsystem trieurSubsystem) {
        addCommands(
                new MoulinNextShoot(trieurSubsystem),
                new MoulinNextShoot(trieurSubsystem),
                new MoulinNextShoot(trieurSubsystem));
    }
}
