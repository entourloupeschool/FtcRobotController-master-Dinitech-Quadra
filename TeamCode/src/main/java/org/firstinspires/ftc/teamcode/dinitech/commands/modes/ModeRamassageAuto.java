package org.firstinspires.ftc.teamcode.dinitech.commands.modes;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextNextLoose;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextNextVeryLoose;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenWaitTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyTrieurForPick;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TryDetectArtefact;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ModeRamassageAuto extends SequentialCommandGroup {

    /**
     * Creates a new ModeRamassage command group.
     *
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     * @param gamepadSubsystem  The gamepad subsystem, passed down to child commands for haptic feedback.
     */
    public ModeRamassageAuto(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem,
                             GamepadSubsystem gamepadSubsystem, int detectArtefactTimeout) {
        addCommands(
                new ReadyTrieurForPick(trieurSubsystem),

                // First, run the detection process
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem, detectArtefactTimeout),
                new ConditionalCommand(
                        new MoulinNextNextVeryLoose(trieurSubsystem),
                        new InstantCommand(), // Do nothing on timeout
                        trieurSubsystem::isArtefactInTrieur
                ),
                // First, run the detection process
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem, detectArtefactTimeout),
                new ConditionalCommand(
                        new MoulinNextNextVeryLoose(trieurSubsystem),
                        new InstantCommand(), // Do nothing on timeout
                        trieurSubsystem::isArtefactInTrieur
                ),
                // First, run the detection process
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem, detectArtefactTimeout),
                new ConditionalCommand(
                        new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem),
                        new InstantCommand(),
                        trieurSubsystem::wantsMotifShoot),
                new OpenWaitTrappe(trieurSubsystem)
        );
    }
}
