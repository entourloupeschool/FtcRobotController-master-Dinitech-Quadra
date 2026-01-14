package org.firstinspires.ftc.teamcode.dinitech.commands.modes;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNext;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe.CloseTrappe;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.ArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.commands.groups.AutomaticArtefactPickAway;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

/**
 * A command group that handles the artifact collection mode of the robot.
 */
public class ModeRamassage extends ConditionalCommand {

    /**
     * Creates a new ModeRamassage command group.
     *
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     * @param chargeurSubsystem The intake subsystem for running the intake motor.
     * @param gamepadSubsystem  The gamepad subsystem, passed down to child commands for haptic feedback.
     */
    public ModeRamassage(TrieurSubsystem trieurSubsystem, ChargeurSubsystem chargeurSubsystem,
                         GamepadSubsystem gamepadSubsystem) {
        super(
                // if condition is true.
                new SequentialCommandGroup(
                    new AutomaticArtefactPickAway(trieurSubsystem, chargeurSubsystem, gamepadSubsystem),
                    new CloseTrappe(trieurSubsystem)),

                // if condition is false.
                new SequentialCommandGroup(
                    new CloseTrappe(trieurSubsystem),
                    new MoulinNext(trieurSubsystem),
                    new AutomaticArtefactPickAway(trieurSubsystem, chargeurSubsystem, gamepadSubsystem)),

                // Condition.
                () -> Moulin.isStoragePosition(trieurSubsystem.getMoulinPosition())
        );
    }
}
