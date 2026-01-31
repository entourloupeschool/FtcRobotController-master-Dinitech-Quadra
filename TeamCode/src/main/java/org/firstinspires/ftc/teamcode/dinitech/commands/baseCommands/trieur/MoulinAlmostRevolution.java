package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

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
public class MoulinAlmostRevolution extends MoulinToPositionVeryLoose {
    /**
     * Creates a new MoulinAlmostRevolution command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public MoulinAlmostRevolution(TrieurSubsystem trieurSubsystem) {
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
        if (Moulin.isStoragePosition(currentPosition)){
            super.moulinTargetPosition = trieurSubsystem.getNNextMoulinPosition(currentPosition, Moulin.TOTAL_POSITIONS-1);
        } else {
            super.moulinTargetPosition = trieurSubsystem.getNNextMoulinPosition(currentPosition, Moulin.TOTAL_POSITIONS-2);
        }

        // Set the parameters for the parent command.
        super.makeShort = false; // Always rotate forward.

        // Start the movement.
        super.initialize();
    }

}
