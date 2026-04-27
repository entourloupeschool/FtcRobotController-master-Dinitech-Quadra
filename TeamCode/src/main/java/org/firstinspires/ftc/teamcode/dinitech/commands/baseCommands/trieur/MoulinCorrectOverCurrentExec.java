package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.Moulin.OVER_CURRENT_BACKOFF_TICKS;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
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
public class MoulinCorrectOverCurrentExec extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;
    private final GamepadSubsystem gamepadSubsystem;
    private double originalMotorTargetPosition;

    /**
     * Creates a new MoulinCorrectOverCurrent command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public MoulinCorrectOverCurrentExec(TrieurSubsystem trieurSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.trieurSubsystem = trieurSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;

        addRequirements(trieurSubsystem);
    }

    /**
     * Initializes the recovery sequence by backing off the motor.
     */
    @Override
    public void initialize() {
        // Save the distance remaining to the original target.
        originalMotorTargetPosition = trieurSubsystem.getMoulinEncoderTargetPosition();

        // Back the motor off by reducing the target position.
        trieurSubsystem.resetMoulinEncoderTarget();
    }

    @Override
    public void execute(){
        trieurSubsystem.incrementMoulinEncoderTargetPosition(OVER_CURRENT_BACKOFF_TICKS/10);
    }

    /**
     * The command is finished once the back-off movement is complete.
     *
     * @return True if the motor has stopped.
     */
    @Override
    public boolean isFinished() {
        return !gamepadSubsystem.getOperator().circle.get();
    }

    /**
     * Restores the original target and reschedules the original command.
     *
     * @param interrupted Whether the correction command itself was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        // Restore the original target by adding back the saved remaining distance.

        if (!interrupted)trieurSubsystem.setMoulinEncoderTargetPosition(originalMotorTargetPosition);

    }
}
