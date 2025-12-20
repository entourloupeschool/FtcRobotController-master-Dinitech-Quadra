package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.velocityTrieur;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.VelocityTrieurSubsystem;

/**
 * A command that rotates the moulin one step forward, designed for the {@link VelocityTrieurSubsystem}.
 * <p>
 * This command extends {@link MoulinToPosition}. Instead of a fixed target, it dynamically
 * determines the target position in its {@code initialize()} method by calculating the next
 * sequential position from the moulin's current state. This ensures the command is always
 * context-aware.
 * <p>
 * It always rotates in the positive (forward) direction.
 */
public class MoulinNext extends MoulinToPosition {

    /**
     * Creates a new MoulinNext command for the velocity-controlled sorter.
     *
     * @param trieurSubsystem The {@link VelocityTrieurSubsystem} to control.
     */
    public MoulinNext(VelocityTrieurSubsystem trieurSubsystem) {
        // The actual target position is determined at execution time.
        super(trieurSubsystem, 0, false);
    }

    /**
     * Dynamically calculates the target position and starts the rotation.
     */
    @Override
    public void initialize() {
        // Calculate the target position based on the current state.
        int currentPosition = trieurSubsystem.getMoulinPosition();
        moulinTargetPosition = trieurSubsystem.getNNextMoulinPosition(currentPosition, 1);
        makeShort = false;

        // Call the parent's initialize() to start the movement.
        super.initialize();
    }
}
