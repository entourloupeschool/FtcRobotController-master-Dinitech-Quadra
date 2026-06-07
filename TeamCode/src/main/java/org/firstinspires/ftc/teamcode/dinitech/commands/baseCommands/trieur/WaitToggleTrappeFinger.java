package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.finger.CloseFinger;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Trappe;

public class WaitReadyShootTrappeFinger extends ConditionalCommand {
    public WaitReadyShootTrappeFinger(TrieurSubsystem trieurSubsystem) {
        super(
                new SequentialCommandGroup(
                        new OpenTrappe(trieurSubsystem),
                        new CloseFinger(trieurSubsystem),
                        new WaitCommand(Trappe.TRAPPE_OPEN_TIME)),
                new InstantCommand(),
                ()->!trieurSubsystem.isTrappeOpen() || trieurSubsystem.isFingerOpen());
    }
}
