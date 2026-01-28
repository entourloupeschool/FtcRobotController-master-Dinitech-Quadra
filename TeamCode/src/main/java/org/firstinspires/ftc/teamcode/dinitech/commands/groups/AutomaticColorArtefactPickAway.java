package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command group that automates the process of picking up artifacts until the sorter is full.
 * <p>
 * This command uses a {@link ParallelDeadlineGroup} to manage a continuous intake process. The
 * entire group runs until the {@link TrieurSubsystem} reports that it is full, which serves as
 * the deadline.
 * <p>
 * The group runs two commands in parallel:
 * <ol>
 *     <li>A {@link StartEndCommand} that runs the intake motor (chargeur) at a constant power.
 *     When the deadline is met (the sorter is full), this command automatically stops the motor.</li>
 *     <li>A {@link RepeatCommand} that continuously executes the {@link ArtefactPickAway} sequence.
 *     This repeatedly detects and stores artifacts as they are collected by the intake.</li>
 * </ol>
 */
public class AutomaticColorArtefactPickAway extends ParallelDeadlineGroup {

    /**
     * Creates a new AutomaticArtefactPickAway command.
     *
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     * @param gamepadSubsystem  The gamepad subsystem, passed down to child commands for haptic feedback.
     */
    public AutomaticColorArtefactPickAway(TrieurSubsystem trieurSubsystem,
                                          GamepadSubsystem gamepadSubsystem) {
        super(
                // The deadline for the group: stop when the sorter is full.
                new WaitUntilCommand(trieurSubsystem::getIsFull),

                // Command 2 (runs in parallel): Repeatedly detect and store artifacts.
                new RepeatCommand(new ColorArtefactPickAway(trieurSubsystem, gamepadSubsystem))
        );
    }
}
