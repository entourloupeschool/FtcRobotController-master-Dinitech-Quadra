package org.firstinspires.ftc.teamcode.dinitech.commands.modes;


import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MODE_RAMASSAGE_AUTO_TIMEOUT;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.MaxPowerChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.chargeur.StopChargeur;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNextNextLoose;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ReadyTrieurForPick;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ModeRamassageAuto extends SequentialCommandGroup {

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
                new ParallelCommandGroup(
                        new ReadyTrieurForPick(trieurSubsystem),
                        new MaxPowerChargeur(chargeurSubsystem)),
                new ArtefactPickAway(trieurSubsystem, gamepadSubsystem),
                new MoulinNextNextLoose(trieurSubsystem),
                new ArtefactPickAway(trieurSubsystem, gamepadSubsystem),
                new MoulinNextNextLoose(trieurSubsystem),
                new ArtefactPickAway(trieurSubsystem, gamepadSubsystem),
                new StopChargeur(chargeurSubsystem)
        );
    }
}
