package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.WaitOpenTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class PrepShootTrieur extends ConditionalCommand {

    public PrepShootTrieur(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem){
        super(
                new SequentialCommandGroup(
                        new ReadyMotifRumble(trieurSubsystem, visionSubsystem, gamepadSubsystem),
                        new WaitOpenTrappe(trieurSubsystem)),
                new WaitOpenTrappe(trieurSubsystem),
                trieurSubsystem::wantsMotifShoot
        );
    }
    public PrepShootTrieur(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem){
        super(
                new SequentialCommandGroup(
                        new ReadyMotif(trieurSubsystem, visionSubsystem),
                        new WaitOpenTrappe(trieurSubsystem)),
                new WaitOpenTrappe(trieurSubsystem),
                trieurSubsystem::wantsMotifShoot
        );
    }
}
