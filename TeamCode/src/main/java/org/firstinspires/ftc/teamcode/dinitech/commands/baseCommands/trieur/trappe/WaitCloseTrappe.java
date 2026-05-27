package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Trappe;

public class WaitCloseTrappe extends ConditionalCommand {
    public WaitCloseTrappe(TrieurSubsystem trieurSubsystem) {
        super(
                new SequentialCommandGroup(
                        new CloseTrappe(trieurSubsystem),
                        new WaitCommand(Trappe.TRAPPE_CLOSE_TIME)),
                new InstantCommand(),
                trieurSubsystem::isTrappeOpen);
    }
}
