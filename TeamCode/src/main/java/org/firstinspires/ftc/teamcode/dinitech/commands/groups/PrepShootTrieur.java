package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

public class PrepShootTrieur extends ConditionalCommand {

    public PrepShootTrieur(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem, GamepadSubsystem gamepadSubsystem){
        super(
                new ReadyMotifRumble(trieurSubsystem, visionSubsystem, gamepadSubsystem),
                new InstantCommand(),
                trieurSubsystem::wantsMotifShoot
        );
    }
    public PrepShootTrieur(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem){
        super(
                new ReadyMotif(trieurSubsystem, visionSubsystem),
                new InstantCommand(),
                trieurSubsystem::wantsMotifShoot
        );
    }
}
