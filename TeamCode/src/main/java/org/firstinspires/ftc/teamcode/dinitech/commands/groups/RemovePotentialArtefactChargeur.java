
package org.firstinspires.ftc.teamcode.dinitech.commands.groups;


import static org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem.INVERSE_MAX_POWER_DURATION_RAMASSAGE_AUTO;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.InverseMaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextStorage;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.WaitCloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class RemovePotentialArtefactChargeur extends SequentialCommandGroup {

    public RemovePotentialArtefactChargeur(TrieurSubsystem trieurSubsystem, ChargeurSubsystem chargeurSubsystem, boolean shouldInversePowerChargeur) {
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InverseMaxPowerChargeur(chargeurSubsystem),
                                new WaitCommand(INVERSE_MAX_POWER_DURATION_RAMASSAGE_AUTO)),
                        new InstantCommand(),
                        ()->shouldInversePowerChargeur && trieurSubsystem.isFull()),
                new StopChargeur(chargeurSubsystem)
        );
    }
}
