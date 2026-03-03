package org.firstinspires.ftc.teamcode.dinitech.commands.modes;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextNextVeryLoose;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.ReadyMotif;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe.OpenWaitTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyTrieurForPick;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TryDetectArtefact;
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
        TryDetectArtefact detectArtefact1 = new TryDetectArtefact(trieurSubsystem, gamepadSubsystem, detectArtefactTimeout);
        TryDetectArtefact detectArtefact2 = new TryDetectArtefact(trieurSubsystem, gamepadSubsystem, detectArtefactTimeout);
        TryDetectArtefact detectArtefact3 = new TryDetectArtefact(trieurSubsystem, gamepadSubsystem, detectArtefactTimeout);

        addCommands(
                new ReadyTrieurForPick(trieurSubsystem),
                detectArtefact1,
                new ConditionalCommand(
                        new InstantCommand(),
                        new SequentialCommandGroup(
                                new MoulinNextNextVeryLoose(trieurSubsystem),
                                detectArtefact2,
                                new ConditionalCommand(
                                        new InstantCommand(),
                                        new SequentialCommandGroup(
                                                new MoulinNextNextVeryLoose(trieurSubsystem),
                                                detectArtefact3),
                                        detectArtefact2::endedByTimeout)),
                        detectArtefact1::endedByTimeout),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ReadyMotif(trieurSubsystem, visionSubsystem, gamepadSubsystem),
                                new OpenWaitTrappe(trieurSubsystem)),
                        new OpenWaitTrappe(trieurSubsystem),
                        trieurSubsystem::wantsMotifShoot)
        );
    }
}
