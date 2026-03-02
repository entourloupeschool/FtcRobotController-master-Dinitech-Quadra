package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

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
public class MoulinNextShootSuper extends MoulinToPositionVeryLoose {

    /**
     * Creates a new MoulinNext command.
     *
     * @param trieurSubsystem The sorter subsystem that controls the moulin.
     */
    public MoulinNextShootSuper(TrieurSubsystem trieurSubsystem) {
        // The actual target position is determined at execution time.
        super(trieurSubsystem, 0, false);
    }

    /**
     * Dynamically calculates the target position and starts the rotation.
     * This method is called once when the command is scheduled.
     */
    @Override
    public void initialize() {
        int currentPos = super.trieurSubsystem.getMoulinPosition();
        super.moulinTargetPosition = -1;

        for (int i = currentPos; i < currentPos + 6; i++) {
            int oneIndexedPos = i % Moulin.TOTAL_POSITIONS;
            if (trieurSubsystem.getMoulinStoragePositionColor(oneIndexedPos) != TrieurSubsystem.ArtifactColor.NONE) {
                super.moulinTargetPosition = oneIndexedPos;
                break;
            }
        }
        super.makeShort = false; // Always rotate forward.

        if (super.moulinTargetPosition == -1) {this.cancel();}
        else {super.initialize();}
    }

    @Override
    public void end(boolean interrupted) {
        trieurSubsystem.clearMoulinStoragePositionColor(super.moulinTargetPosition);
    }
}
