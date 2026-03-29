package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem.TRAPPE_OPEN_TIME;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class WaitOpenTrappe extends ConditionalCommand {
    public WaitOpenTrappe(TrieurSubsystem trieurSubsystem) {
        super(
            new InstantCommand(),
            new SequentialCommandGroup(
                    new OpenTrappe(trieurSubsystem),
                    new WaitCommand(TRAPPE_OPEN_TIME)),
            trieurSubsystem::isTrappeOpen);
    }
}
