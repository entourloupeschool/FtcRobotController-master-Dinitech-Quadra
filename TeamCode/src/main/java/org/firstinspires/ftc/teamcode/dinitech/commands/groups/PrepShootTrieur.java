package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenWaitTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class PrepShootTrieur extends ConditionalCommand {

    public PrepShootTrieur(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem){
        super(
                new SequentialCommandGroup(
                        new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem),
                        new OpenWaitTrappe(trieurSubsystem)),
                new OpenWaitTrappe(trieurSubsystem),
                trieurSubsystem::wantsMotifShoot
        );
    }
}
