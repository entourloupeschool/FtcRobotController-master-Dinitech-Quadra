package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command that rotates the moulin two steps forward.
 * <p>
 * This command extends {@link MoulinToPosition} and, like {@link MoulinNext}, it
 * dynamically determines its target position at runtime. It calculates the position
 * two steps forward from the current moulin position and rotates the mechanism to
 * that spot, always moving in the positive (forward) direction.
 * <p>
 * This is typically used after picking up an artifact to move it out of the intake area
 * and prepare the next empty slot.
 */
public class MoulinNextNextLoose extends MoulinToPositionLoose {

    /**
     * Creates a new MoulinNextNext command.
     *
     * @param trieurSubsystem The sorter subsystem that controls the moulin.
     */
    public MoulinNextNextLoose(TrieurSubsystem trieurSubsystem){
        // The actual target position is determined at execution time.
        super(trieurSubsystem, 0, false);
    }

    /**
     * Dynamically calculates the target position and starts the rotation.
     * This method is called once when the command is scheduled.
     */
    @Override
    public void initialize() {
        // Calculate the target position two steps forward from the current state.
        super.moulinTargetPosition = trieurSubsystem.getNNextMoulinPosition(trieurSubsystem.getMoulinPosition(), 2);
        super.makeShort = false; // Always rotate forward.

        // Call the parent's initialize() to start the movement.
        super.initialize();
    }
}
