package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.WAIT_HIGH_SPEED_TRIEUR;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class MoulinHighSpeedRevolution extends SequentialCommandGroup {

    public MoulinHighSpeedRevolution(TrieurSubsystem trieurSubsystem, CommandBase waitCommand, CommandBase waitUntilCommand) {
        addCommands(
            waitUntilCommand,
            new MoulinNextShoot(trieurSubsystem),
            waitCommand,
            waitUntilCommand,
            new MoulinNextNextVeryLoose(trieurSubsystem),
            waitCommand,
            waitUntilCommand,
            new MoulinNextNextVeryLoose(trieurSubsystem));
    }

    public MoulinHighSpeedRevolution(TrieurSubsystem trieurSubsystem) {
        addCommands(
                new MoulinNextShoot(trieurSubsystem),
                new MoulinNextNextVeryLoose(trieurSubsystem),
                new MoulinNextNextVeryLoose(trieurSubsystem));
    }
}
