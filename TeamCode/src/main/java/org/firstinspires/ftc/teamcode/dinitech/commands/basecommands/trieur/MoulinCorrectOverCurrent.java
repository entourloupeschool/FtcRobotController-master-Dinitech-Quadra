package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.OVER_CURRENT_BACKOFF_TICKS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.POWER_MOULIN_ROTATION;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command that attempts to recover the moulin from an over-current (jam) condition.
 * <p>
 * This command is scheduled by other moulin commands (like {@link MoulinRotate}) when an
 * over-current event is detected. It performs a recovery sequence:
 * <ol>
 *     <li>Saves the remaining distance to the original target.</li>
 *     <li>Backs the motor off by a small, predefined amount to relieve pressure.</li>
 *     <li>Waits for the back-off movement to complete.</li>
 *     <li>Restores the original target position and re-schedules the command that was
 *     initially interrupted, allowing it to try again.</li>
 * </ol>
 */
public class MoulinCorrectOverCurrent extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;
    private final CommandBase originalCommand;
    private int savedRemainingDistance;

    /**
     * Creates a new MoulinCorrectOverCurrent command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     * @param originalCommand The command that was interrupted by the over-current event.
     */
    public MoulinCorrectOverCurrent(TrieurSubsystem trieurSubsystem, CommandBase originalCommand) {
        this.trieurSubsystem = trieurSubsystem;
        this.originalCommand = originalCommand;

        addRequirements(trieurSubsystem);
    }

    /**
     * Initializes the recovery sequence by backing off the motor.
     */
    @Override
    public void initialize() {
        // Save the distance remaining to the original target.
        savedRemainingDistance = Math.abs(trieurSubsystem.getMoulinMotorRemainingDistance());

        // Back the motor off by reducing the target position.
        trieurSubsystem.incrementMoulinTargetPosition(-OVER_CURRENT_BACKOFF_TICKS);
        trieurSubsystem.setMoulinPower(POWER_MOULIN_ROTATION);
    }

    /**
     * The command is finished once the back-off movement is complete.
     *
     * @return True if the motor has stopped.
     */
    @Override
    public boolean isFinished() {
        return trieurSubsystem.shouldMoulinStopPower();
    }

    /**
     * Restores the original target and reschedules the original command.
     *
     * @param interrupted Whether the correction command itself was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        // Restore the original target by adding back the saved remaining distance.
        trieurSubsystem.incrementMoulinTargetPosition(savedRemainingDistance);
        trieurSubsystem.setMoulinPower(POWER_MOULIN_ROTATION);

        // If the correction wasn't interrupted, reschedule the original command to try again.
        if (!interrupted && originalCommand != null) {
            originalCommand.schedule();
        }
    }
}
