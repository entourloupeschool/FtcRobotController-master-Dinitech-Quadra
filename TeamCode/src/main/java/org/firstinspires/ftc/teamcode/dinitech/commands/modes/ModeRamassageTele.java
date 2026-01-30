package org.firstinspires.ftc.teamcode.dinitech.commands.modes;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_AUTO_TIMEOUT;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_TELE_TIMEOUT;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNextNextLoose;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNextNextVeryLoose;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TryDetectArtefact;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ModeRamassageTele extends SequentialCommandGroup {

    /**
     * Creates a new ModeRamassage command group.
     *
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     * @param gamepadSubsystem  The gamepad subsystem, passed down to child commands for haptic feedback.
     */
    public ModeRamassageTele(TrieurSubsystem trieurSubsystem, ChargeurSubsystem chargeurSubsystem,
                             GamepadSubsystem gamepadSubsystem) {
        addCommands(
                // First, run the detection process
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem, MODE_RAMASSAGE_TELE_TIMEOUT),
                new ConditionalCommand(
                        new MoulinNextNextLoose(trieurSubsystem),
                        new InstantCommand(), // Do nothing on timeout
                        trieurSubsystem::isArtefactInTrieur
                ),
                // First, run the detection process
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem, MODE_RAMASSAGE_TELE_TIMEOUT),
                new ConditionalCommand(
                        new MoulinNextNextLoose(trieurSubsystem),
                        new InstantCommand(), // Do nothing on timeout
                        trieurSubsystem::isArtefactInTrieur
                ),
                // First, run the detection process
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem, MODE_RAMASSAGE_TELE_TIMEOUT),
                new ConditionalCommand(
                        new StopChargeur(chargeurSubsystem),
                        new InstantCommand(),
                        trieurSubsystem::getIsFull)
        );
    }
}
