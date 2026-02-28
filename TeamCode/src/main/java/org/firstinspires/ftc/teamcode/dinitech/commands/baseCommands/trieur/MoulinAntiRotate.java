package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.MOULIN_ROTATE_SPEED_CONTINUOUS;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

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
    private final GamepadWrapper operator;

    /**
     * Creates a new MoulinAntiRotate command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public MoulinAntiRotate(TrieurSubsystem trieurSubsystem, GamepadSubsystem gamepadSubsystem) {
        this.trieurSubsystem = trieurSubsystem;
        this.operator = gamepadSubsystem.getOperator();
        addRequirements(trieurSubsystem);
    }

    /**
     * Initializes the rotation by setting a new negative target and applying motor power.
     */
    @Override
    public void execute() {
        double leftTriggerValue = operator.getLeftTriggerValue();

        trieurSubsystem.incrementMoulinTargetPosition(- leftTriggerValue * leftTriggerValue * MOULIN_ROTATE_SPEED_CONTINUOUS);
    }
}
