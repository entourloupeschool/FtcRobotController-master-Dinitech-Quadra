package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.finger.CloseFinger;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.finger.OpenFinger;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.CloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Trappe;

public class WaitToggleTrappeFinger extends ConditionalCommand {
    public WaitToggleTrappeFinger(TrieurSubsystem trieurSubsystem) {
        super(
                new SequentialCommandGroup(
                        new InstantCommand(()->{
                            trieurSubsystem.openTrappe();
                            trieurSubsystem.closeFinger();
                        }, trieurSubsystem),
                        new WaitCommand(Trappe.TRAPPE_OPEN_TIME)),
                new SequentialCommandGroup(
                        new InstantCommand(()->{
                            trieurSubsystem.closeTrappe();
                            trieurSubsystem.openFinger();
                        }, trieurSubsystem),
                        new WaitCommand(Trappe.TRAPPE_CLOSE_TIME)),
                ()->!trieurSubsystem.isTrappeOpen() || trieurSubsystem.isFingerOpen());
    }
}
