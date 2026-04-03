package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Trappe;

public class WaitOpenTrappe extends ConditionalCommand {
    public WaitOpenTrappe(TrieurSubsystem trieurSubsystem) {
        super(
            new InstantCommand(),
            new SequentialCommandGroup(
                    new OpenTrappe(trieurSubsystem),
                    new WaitCommand(Trappe.TRAPPE_OPEN_TIME)),
            trieurSubsystem::isTrappeOpen);
    }
}
