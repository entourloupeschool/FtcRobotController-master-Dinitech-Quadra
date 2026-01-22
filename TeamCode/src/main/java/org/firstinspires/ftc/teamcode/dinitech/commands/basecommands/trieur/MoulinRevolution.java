package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

/**
 * A command that performs a nearly full revolution of the moulin to cycle through all positions.
 * <p>
 * This command extends {@link MoulinToPosition} and dynamically calculates its target
 * to be {@code TOTAL_POSITIONS - 1} steps forward from the current position. It always
 * rotates in the positive (forward) direction.
 * <p>
 * This is typically used in a shooting sequence (like {@link org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootRevolution})
 * to feed all loaded artifacts into the shooter mechanism sequentially.
 */
public class MoulinRevolution extends MoulinToPosition {

    /**
     * Creates a new MoulinRevolution command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public MoulinRevolution(TrieurSubsystem trieurSubsystem) {
        // The actual target position is determined at execution time.
        super(trieurSubsystem, 0, false);
    }

    /**
     * Dynamically calculates the target position for the revolution and starts the rotation.
     */
    @Override
    public void initialize() {
        // Calculate the target position to be a nearly full circle forward.
        int currentPosition = trieurSubsystem.getMoulinPosition();
        moulinTargetPosition = trieurSubsystem.getNNextMoulinPosition(currentPosition, Moulin.TOTAL_POSITIONS);
        makeShort = false; // Always rotate forward for a full revolution.

        // Call the parent's initialize() to start the movement.
        super.initialize();
    }
}
