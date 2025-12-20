package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.velocityTrieur;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.VelocityTrieurSubsystem;

/**
 * A command that rotates the moulin two steps forward, designed for the {@link VelocityTrieurSubsystem}.
 * <p>
 * This command extends {@link MoulinToPosition}. It dynamically determines its target
 * position in the {@code initialize()} method by calculating the position two steps forward
 * from the moulin's current state. This is typically used after intaking an artifact to
 * move it away from the intake mechanism and prepare the next empty slot.
 * <p>
 * It always rotates in the positive (forward) direction.
 */
public class MoulinNextNext extends MoulinToPosition {

    /**
     * Creates a new MoulinNextNext command for the velocity-controlled sorter.
     *
     * @param trieurSubsystem The {@link VelocityTrieurSubsystem} to control.
     */
    public MoulinNextNext(VelocityTrieurSubsystem trieurSubsystem){
        // The actual target position is determined at execution time.
        super(trieurSubsystem, 0, false);
    }

    /**
     * Dynamically calculates the target position and starts the rotation.
     */
    @Override
    public void initialize() {
        // Calculate the target position two steps forward from the current state.
        int currentPosition = trieurSubsystem.getMoulinPosition();
        moulinTargetPosition = trieurSubsystem.getNNextMoulinPosition(currentPosition, 2);
        makeShort = false;

        // Call the parent's initialize() to start the movement.
        super.initialize();
    }
}
