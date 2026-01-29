package org.firstinspires.ftc.teamcode.dinitech.commands.modes;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe.CloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.AutomaticArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ModeRamassageAuto extends ConditionalCommand {

    /**
     * Creates a new ModeRamassage command group.
     *
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     * @param chargeurSubsystem The intake subsystem for running the intake motor.
     * @param gamepadSubsystem  The gamepad subsystem, passed down to child commands for haptic feedback.
     */
    public ModeRamassageAuto(TrieurSubsystem trieurSubsystem, ChargeurSubsystem chargeurSubsystem,
                             GamepadSubsystem gamepadSubsystem) {
        super(
                // if condition is true.
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new MaxPowerChargeur(chargeurSubsystem),
                                new CloseTrappe(trieurSubsystem)
                        ),
                        new AutomaticArtefactPickAway(trieurSubsystem, gamepadSubsystem),
                        new StopChargeur(chargeurSubsystem)),

                // if condition is false.
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new MaxPowerChargeur(chargeurSubsystem),
                                new CloseTrappe(trieurSubsystem)
                        ),
                        new MoulinNext(trieurSubsystem),
                        new AutomaticArtefactPickAway(trieurSubsystem, gamepadSubsystem),
                        new StopChargeur(chargeurSubsystem)),
                
                // Condition.
                () -> Moulin.isStoragePosition(trieurSubsystem.getMoulinPosition())
        );
    }
}
