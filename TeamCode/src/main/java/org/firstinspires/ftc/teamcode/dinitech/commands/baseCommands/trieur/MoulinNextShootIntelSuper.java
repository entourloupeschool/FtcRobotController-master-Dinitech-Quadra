package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.INTERVALLE_TICKS_MOULIN;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_POSITION_VERY_LOOSE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.SPEED_MARGIN_SUPER_INTEL;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin;

/**
 * A command that rotates the moulin one step forward to the next sequential position.
 * <p>
 * This command extends {@link MoulinToPositionMargin}. Unlike its parent, the target position
 * is not specified at construction. Instead, it is dynamically determined in the
 * {@code initialize()} method by getting the current moulin position and calculating the
 * next one. This ensures the command always moves to the correct next slot relative
 * to the state of the robot when the command is executed.
 * <p>
 * This command always rotates in the positive (forward) direction.
 */
public class MoulinNextShootIntel extends MoulinToPositionMargin {
    private final ShooterSubsystem shooterSubsystem;
    private boolean hasLaunched;
    /**
     * Creates a new MoulinNext command.
     *
     * @param trieurSubsystem The sorter subsystem that controls the moulin.
     */
    public MoulinNextShootIntel(TrieurSubsystem trieurSubsystem, ShooterSubsystem shooterSubsystem) {
        // The actual target position is determined at execution time.
        super(trieurSubsystem, -1, false, MOULIN_POSITION_VERY_LOOSE_TOLERANCE);
        this.shooterSubsystem = shooterSubsystem;
        this.hasLaunched = false;
    }

    /**
     * Dynamically calculates the target position and starts the rotation.
     * This method is called once when the command is scheduled.
     */
    @Override
    public void initialize() {
        hasLaunched = false;

        int currentPos = trieurSubsystem.getMoulinPosition();

        for (int i = 1; i < Moulin.TOTAL_POSITIONS + 1; i++) {
            int previousI = Moulin.getNPreviousPosition(currentPos, i);
            TrieurSubsystem.ArtifactColor color = trieurSubsystem.getMoulinStoragePositionColor(previousI);

            if (color == TrieurSubsystem.ArtifactColor.GREEN || color == TrieurSubsystem.ArtifactColor.PURPLE) {
                super.moulinTargetPosition = Moulin.getOppositePosition(previousI);
                break;
            }
        }

        if (super.moulinTargetPosition == -1){
            super.initialize();
            hasLaunched = true;
        }
    }

    @Override
    public void execute(){
        if (shooterSubsystem.isAroundTargetSpeed(SPEED_MARGIN_SUPER_INTEL) && !hasLaunched){
            super.initialize();
            hasLaunched = true;
        }
    }

    @Override
    public boolean isFinished(){
        return hasLaunched && super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (super.moulinTargetPosition != -1) trieurSubsystem.clearMoulinStoragePositionColor(Moulin.getOppositePosition(super.moulinTargetPosition));
        super.end(interrupted);

    }
}
