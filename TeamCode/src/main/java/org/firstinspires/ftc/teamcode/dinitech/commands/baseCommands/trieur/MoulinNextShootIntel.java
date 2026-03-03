package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN_SUPER_INTEL;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
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
public class MoulinNextShootIntel extends MoulinToPositionVeryLoose {

    private final ShooterSubsystem shooterSubsystem;
    private boolean hasLaunched;
    /**
     * Creates a new MoulinNext command.
     *
     * @param trieurSubsystem The sorter subsystem that controls the moulin.
     */
    public MoulinNextShootIntel(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem) {
        // The actual target position is determined at execution time.
        super(trieurSubsystem, 0, false);
        this.shooterSubsystem = shooterSubsystem;
        this.hasLaunched = false;
    }

    /**
     * Dynamically calculates the target position and starts the rotation.
     * This method is called once when the command is scheduled.
     */
    @Override
    public void initialize() {
        int currentPos = trieurSubsystem.getMoulinPosition();
        super.moulinTargetPosition = -1;
        this.hasLaunched = false;

        for (int i = 1; i < Moulin.TOTAL_POSITIONS + 1; i++) {
            int oneIndexedPos = Moulin.getNPreviousPosition(currentPos, i);
            if (trieurSubsystem.getMoulinStoragePositionColor(oneIndexedPos) != TrieurSubsystem.ArtifactColor.NONE) {
                super.moulinTargetPosition = Moulin.getOppositePosition(oneIndexedPos);
                break;
            }
        }
        super.makeShort = false;
    }

    @Override
    public void execute(){
        if (super.moulinTargetPosition != -1 && shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN_SUPER_INTEL) && !hasLaunched){
            super.initialize();
            hasLaunched = true;
        }
    }

    @Override
    public boolean isFinished(){
        return super.moulinTargetPosition == -1 || (hasLaunched && super.isFinished());
    }

    @Override
    public void end(boolean interrupted) {
        if (super.moulinTargetPosition != -1){trieurSubsystem.clearMoulinStoragePositionColor(Moulin.getOppositePosition(super.moulinTargetPosition));}
        super.end(interrupted);
    }
}
