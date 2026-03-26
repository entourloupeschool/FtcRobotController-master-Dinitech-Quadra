package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem.MOULIN_POSITION_VERY_LOOSE_TOLERANCE;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command that rotates the moulin one step backward to the previous sequential position.
 * <p>
 * This command extends {@link MoulinToPositionMargin}. Similar to {@link MoulinNext}, it determines
 * its target position dynamically in the {@code initialize()} method. It calculates the position
 * one step backward from the current state and rotates the mechanism using the shortest path
 * to get there.
 */
public class MoulinPrevious extends MoulinToPositionMargin {

    /**
     * Creates a new MoulinPrevious command.
     *
     * @param trieurSubsystem The sorter subsystem that controls the moulin.
     */
    public MoulinPrevious(TrieurSubsystem trieurSubsystem){
        // The target is set dynamically, but we default to using the shortest path.
        super(trieurSubsystem, -1, true, MOULIN_POSITION_VERY_LOOSE_TOLERANCE);
    }

    /**
     * Dynamically calculates the target position and starts the rotation.
     * This method is called once when the command is scheduled.
     */
    @Override
    public void initialize() {
        // Calculate the target position based on the current state of the moulin.
        int currentPosition = trieurSubsystem.getMoulinPosition();
        moulinTargetPosition = trieurSubsystem.getNPreviousMoulinPosition(currentPosition, 1);

        // Call the parent's initialize() to start the movement.
        super.initialize();
    }
}
