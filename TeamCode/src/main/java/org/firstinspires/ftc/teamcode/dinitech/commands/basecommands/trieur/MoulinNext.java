package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command that rotates the moulin one step forward to the next sequential position.
 * <p>
 * This command extends {@link MoulinToPosition}. Unlike its parent, the target position
 * is not specified at construction. Instead, it is dynamically determined in the
 * {@code initialize()} method by getting the current moulin position and calculating the
 * next one. This ensures the command always moves to the correct next slot relative
 * to the state of the robot when the command is executed.
 * <p>
 * This command always rotates in the positive (forward) direction.
 */
public class MoulinNext extends MoulinToPosition {

    /**
     * Creates a new MoulinNext command.
     *
     * @param trieurSubsystem The sorter subsystem that controls the moulin.
     */
    public MoulinNext(TrieurSubsystem trieurSubsystem) {
        // The actual target position is determined at execution time.
        super(trieurSubsystem, 0, false);
    }

    /**
     * Dynamically calculates the target position and starts the rotation.
     * This method is called once when the command is scheduled.
     */
    @Override
    public void initialize() {
        // Calculate the target position based on the current state of the moulin.
        int currentPosition = trieurSubsystem.getMoulinPosition();
        int nextPosition = trieurSubsystem.getNNextMoulinPosition(currentPosition, 1);
        
        // Set the parameters for the parent command.
        super.moulinTargetPosition = nextPosition;
        super.makeShort = false; // Always rotate forward.

        // Start the movement.
        super.initialize();
    }
}
