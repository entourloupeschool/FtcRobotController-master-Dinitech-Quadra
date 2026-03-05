package org.firstinspires.ftc.teamcode.dinitech.commands.modes;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.InverseMaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextNextVeryLoose;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.PrepShootTrieur;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyTrieurForPick;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TryDetectArtefact;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ModeRamassageAutoGate extends SequentialCommandGroup {

    /**
     * Creates a new ModeRamassage command group.
     *
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     * @param gamepadSubsystem  The gamepad subsystem, passed down to child commands for haptic feedback.
     */
    public ModeRamassageAutoGate(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem,
                                 GamepadSubsystem gamepadSubsystem, ChargeurSubsystem chargeurSubsystem) {
        addCommands(
                new ReadyTrieurForPick(trieurSubsystem),
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new MoulinNextNextVeryLoose(trieurSubsystem),
                                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem)),
                        new InstantCommand(),
                        trieurSubsystem::getNewRegister),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new MoulinNextNextVeryLoose(trieurSubsystem),
                                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem)),
                        new InstantCommand(),
                        trieurSubsystem::getNewRegister),
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new MoulinNextNextVeryLoose(trieurSubsystem),
                                new InverseMaxPowerChargeur(chargeurSubsystem)),
                        new InstantCommand(),
                        trieurSubsystem::getIsFull),
                new PrepShootTrieur(trieurSubsystem, visionSubsystem, gamepadSubsystem)
        );
    }
}
