package org.firstinspires.ftc.teamcode.dinitech.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinNextNextLoose;
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
public class AutomaticArtefactPickAway extends ConditionalCommand {

    /**
     * Creates a new AutomaticArtefactPickAway command.
     *
     * @param trieurSubsystem   The sorter subsystem, which manages artifact storage and state.
     * @param gamepadSubsystem  The gamepad subsystem, passed down to child commands for haptic feedback.
     */
    public AutomaticArtefactPickAway(TrieurSubsystem trieurSubsystem,
            GamepadSubsystem gamepadSubsystem) {
        super(
            new InstantCommand(),
            buildPickSequence(trieurSubsystem, gamepadSubsystem, 3),
            trieurSubsystem::getIsFull
        );
    }

    /**
     * Recursively builds a pick sequence that repeats up to {@code depth} times,
     * stopping early if the sorter becomes full.
     *
     * @param trieurSubsystem   The sorter subsystem.
     * @param gamepadSubsystem  The gamepad subsystem for haptic feedback.
     * @param depth             The number of pick attempts remaining.
     * @return A command that performs the pick sequence with conditional recursion.
     */
    private static SequentialCommandGroup buildPickSequence(TrieurSubsystem trieurSubsystem,
            GamepadSubsystem gamepadSubsystem, int depth) {
        
        SequentialCommandGroup pickAndAdvance = new SequentialCommandGroup(
                new ArtefactPickAway(trieurSubsystem, gamepadSubsystem),
                new ConditionalCommand(
                        new InstantCommand(),
                        new MoulinNextNextLoose(trieurSubsystem),
                        trieurSubsystem::getIsFull
                )
        );

        if (depth <= 1) {
            // Base case: just do the pick sequence once
            return pickAndAdvance;
        }

        // Recursive case: pick, then conditionally continue if not full
        return new SequentialCommandGroup(
                pickAndAdvance,
                new ConditionalCommand(
                        new InstantCommand(),
                        buildPickSequence(trieurSubsystem, gamepadSubsystem, depth - 1),
                        trieurSubsystem::getIsFull
                )
        );
    }
}
