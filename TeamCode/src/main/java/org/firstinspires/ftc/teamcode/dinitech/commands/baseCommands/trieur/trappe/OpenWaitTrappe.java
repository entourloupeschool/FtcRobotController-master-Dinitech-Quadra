package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.TRAPPE_OPEN_TIME;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

public class OpenWaitTrappe extends SequentialCommandGroup {
    public OpenWaitTrappe(TrieurSubsystem trieurSubsystem) {
        addCommands(
                new OpenTrappe(trieurSubsystem),
                new WaitCommand(TRAPPE_OPEN_TIME));
    }
}
