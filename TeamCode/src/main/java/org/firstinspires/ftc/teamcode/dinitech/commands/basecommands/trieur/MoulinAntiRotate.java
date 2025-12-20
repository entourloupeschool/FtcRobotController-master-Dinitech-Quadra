package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_ROTATE_SPEED_CONTINUOUS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.POWER_MOULIN_ROTATION;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command that rotates the moulin backward continuously until a stop condition is met.
 * <p>
 * This command is the counterpart to {@link MoulinRotate}. It sets a distant target in the
 * negative direction and applies power to rotate the moulin backward. The command will run
 * until it is interrupted, it reaches its target, or an over-current event occurs.
 * <p>
 * Similar to its counterpart, if the command ends due to an over-current event, it schedules a
 * {@link MoulinCorrectOverCurrent} command to attempt to resolve the issue.
 */
public class MoulinAntiRotate extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;

    /**
     * Creates a new MoulinAntiRotate command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public MoulinAntiRotate(TrieurSubsystem trieurSubsystem) {
        this.trieurSubsystem = trieurSubsystem;
        addRequirements(trieurSubsystem);
    }

    /**
     * Initializes the rotation by setting a new negative target and applying motor power.
     */
    @Override
    public void initialize() {
        trieurSubsystem.incrementMoulinTargetPosition(-MOULIN_ROTATE_SPEED_CONTINUOUS);
        trieurSubsystem.setMoulinPower(POWER_MOULIN_ROTATION);
    }

    /**
     * The command is finished when the subsystem indicates that the power should be stopped.
     *
     * @return True if the command should end, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return trieurSubsystem.shouldMoulinStopPower();
    }

    /**
     * Stops the motor and, if necessary, schedules a correction command.
     *
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        trieurSubsystem.setMoulinPower(0);

        // If the command ended naturally due to over-current, schedule a correction command.
        if (!interrupted && trieurSubsystem.isMoulinOverCurrent()) {
            new MoulinCorrectOverCurrent(trieurSubsystem, this).schedule();
        }
    }
}
