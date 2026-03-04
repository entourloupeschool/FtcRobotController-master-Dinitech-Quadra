package org.firstinspires.ftc.teamcode.dinitech.commands.modes;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.MoulinNextNextVeryLoose;
import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.PrepShootTrieur;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyTrieurForPick;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.TryDetectArtefact;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ModeRamassageAutoVintage extends SequentialCommandGroup {

    /**
     * Creates a new ModeRamassage command group.
     *
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     * @param gamepadSubsystem  The gamepad subsystem, passed down to child commands for haptic feedback.
     */
    public ModeRamassageAutoVintage(TrieurSubsystem trieurSubsystem, VisionSubsystem visionSubsystem,
                                    GamepadSubsystem gamepadSubsystem) {
        addCommands(
                new ReadyTrieurForPick(trieurSubsystem),
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem),
                new MoulinNextNextVeryLoose(trieurSubsystem),
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem),
                new MoulinNextNextVeryLoose(trieurSubsystem),
                new TryDetectArtefact(trieurSubsystem, gamepadSubsystem),
                new PrepShootTrieur(trieurSubsystem, visionSubsystem, gamepadSubsystem)
        );
    }
}
