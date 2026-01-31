package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * An instant command that stops the moulin motor.
 * <p>
 * This command sets the moulin motor's power to zero. It also includes logic to handle
 * interruptions: if the command is interrupted (e.g., by another command requiring the
 * subsystem), it resets the motor's target position to its current position. This effectively
 * cancels any ongoing {@code RUN_TO_POSITION} movement and prevents the motor from moving
 * unexpectedly when the interrupting command finishes.
 */
public class StopMoulin extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;

    /**
     * Creates a new StopMoulin command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public StopMoulin(TrieurSubsystem trieurSubsystem){
        this.trieurSubsystem = trieurSubsystem;
        addRequirements(trieurSubsystem);
    }

    /**
     * Sets the motor power to zero when the command is initialized.
     */
    @Override
    public void initialize(){
        trieurSubsystem.setMoulinPower(0);
    }

    /**
     * This command finishes immediately after being initialized.
     *
     * @return Always returns true.
     */
    @Override
    public boolean isFinished() {
        return true;
    }

    /**
     * Resets the motor's target if the command is interrupted.
     *
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted){
        if (interrupted){
            trieurSubsystem.resetTargetMoulinMotor();
        }
    }
}
