package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRAPPE_CLOSE_TIME;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class WaitCloseTrappe extends ConditionalCommand {
    public WaitCloseTrappe(TrieurSubsystem trieurSubsystem) {
        super(
                new SequentialCommandGroup(
                        new CloseTrappe(trieurSubsystem),
                        new WaitCommand(TRAPPE_CLOSE_TIME)),
                new InstantCommand(),
                trieurSubsystem::isTrappeOpen);
    }
}
