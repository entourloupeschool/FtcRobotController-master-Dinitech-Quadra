package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_ROTATE_SPEED_CONTINUOUS;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

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
    private final GamepadWrapper operator;


    /**
     * Creates a new MoulinRotate command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public MoulinRotate(TrieurSubsystem trieurSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.trieurSubsystem = trieurSubsystem;
        this.operator = gamepadSubsystem.getOperator();
        addRequirements(trieurSubsystem);
    }

    /**
     * Initializes the rotation by setting a new target and applying motor power.
     */
    @Override
    public void execute() {
        double rightTriggerValue = operator.getRightTriggerValue();

        trieurSubsystem.incrementMoulinTargetPosition(rightTriggerValue * rightTriggerValue * MOULIN_ROTATE_SPEED_CONTINUOUS);
    }
}
