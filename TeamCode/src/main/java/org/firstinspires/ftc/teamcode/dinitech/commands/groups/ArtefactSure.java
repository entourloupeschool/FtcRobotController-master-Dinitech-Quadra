package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;


public class ArtefactSure extends SequentialCommandGroup {
    public ArtefactSure(TrieurSubsystem trieurSubsystem, ChargeurSubsystem chargeurSubsystem) {
        addCommands(
                new ParallelRaceGroup(
                        new RunCommand(trieurSubsystem::updateColorSensors),
                        new WaitUntilCommand(trieurSubsystem::isArtefactInTrieur),
                        new MaxPowerChargeur(chargeurSubsystem)),
                new StopChargeur(chargeurSubsystem)
        );
    }
}
