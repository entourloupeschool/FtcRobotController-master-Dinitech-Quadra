package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_ROTATE_SPEED_CONTINUOUS;
import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.POWER_MOULIN_ROTATION;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command that rotates the moulin forward continuously until a stop condition is met.
 * <p>
 * This command sets a distant target for the moulin motor and applies power to rotate it.
 * The command will run until one of the following conditions is met:
 * <ul>
 *     <li>The motor reaches its target (unlikely with a large target value).</li>
 *     <li>An over-current event occurs, indicating a potential stall or jam.</li>
 *     <li>The command is interrupted by another command.</li>
 * </ul>
 * If the command ends due to an over-current event, it schedules a {@link MoulinCorrectOverCurrent}
 * command to attempt to resolve the issue.
 */
public class MoulinRotate extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;

    /**
     * Creates a new MoulinRotate command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public MoulinRotate(TrieurSubsystem trieurSubsystem) {
        this.trieurSubsystem = trieurSubsystem;
        addRequirements(trieurSubsystem);
    }

    /**
     * Initializes the rotation by setting a new target and applying motor power.
     */
    @Override
    public void initialize() {
        trieurSubsystem.incrementMoulinTargetPosition(MOULIN_ROTATE_SPEED_CONTINUOUS);
    }

    /**
     * The command is finished when the subsystem indicates that the power should be stopped
     * (e.g., due to over-current or reaching the target).
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
        // If the command ended naturally due to over-current, schedule a correction command.
        if (!interrupted && trieurSubsystem.isMoulinOverCurrent()) {
            new MoulinCorrectOverCurrent(trieurSubsystem, this).schedule();
        }
    }
}
