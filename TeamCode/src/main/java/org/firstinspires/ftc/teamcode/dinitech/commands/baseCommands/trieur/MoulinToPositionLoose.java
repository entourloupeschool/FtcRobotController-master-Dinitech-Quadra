package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A fundamental command for rotating the moulin to a specific target position.
 * <p>
 * This command serves as the base for most moulin rotation commands (e.g., {@link MoulinNext},
 * {@link MoulinPrevious}, {@link MoulinRevolution}). It takes a target position and a pathing
 * option, then commands the {@link TrieurSubsystem} to perform the rotation.
 * <p>
 * The command finishes when the moulin either reaches its target or an over-current event
 * is detected. It also includes logic to stop the motor and reset its target if the command
 * is interrupted, preventing unwanted movement.
 */
public class MoulinToPositionLoose extends CommandBase {
    protected final TrieurSubsystem trieurSubsystem;
    protected int moulinTargetPosition;
    protected boolean makeShort;

    /**
     * Creates a new MoulinToPosition command.
     *
     * @param trieurSubsystem      The sorter subsystem to control.
     * @param moulinTargetPosition The target logical position (1-6) for the moulin.
     * @param makeShort            If true, the moulin will take the shortest path; otherwise, it will rotate forward.
     */
    public MoulinToPositionLoose(TrieurSubsystem trieurSubsystem, int moulinTargetPosition, boolean makeShort) {
        this.trieurSubsystem = trieurSubsystem;
        this.moulinTargetPosition = moulinTargetPosition;
        this.makeShort = makeShort;

        addRequirements(trieurSubsystem);
    }

    /**
     * Initiates the rotation by commanding the subsystem to move to the target position.
     */
    @Override
    public void initialize() {
        trieurSubsystem.moulinToPosition(moulinTargetPosition, makeShort);
    }

    /**
     * The command is finished when the subsystem indicates that motor power should be cut.
     *
     * @return True if the command should end.
     */
    @Override
    public boolean isFinished() {
        return trieurSubsystem.shouldMoulinStopPowerLoose();
    }

    /**
     * Stops the motor at the end of the command and handles interruptions.
     *
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        // If the command was interrupted before completion, reset the motor's target
        // to its current position to prevent further movement.
        if (interrupted) {
            trieurSubsystem.resetTargetMoulinMotor();
        }
    }
}
