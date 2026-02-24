package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * A command that performs a nearly full revolution of the moulin to cycle through all positions.
 * <p>
 * This command extends {@link MoulinToPosition} and dynamically calculates its target
 * to be {@code TOTAL_POSITIONS - 1} steps forward from the current position. It always
 * rotates in the positive (forward) direction.
 * <p>
 * This is typically used in a shooting sequence (like {@link org.firstinspires.ftc.teamcode.dinitech.commands.groups.ShootRevolution})
 * to feed all loaded artifacts into the shooter mechanism sequentially.
 */
public class MoulinRevolution extends CommandBase {
    protected final TrieurSubsystem trieurSubsystem;

    /**
     * Creates a new MoulinRevolution command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public MoulinRevolution(TrieurSubsystem trieurSubsystem) {
        this.trieurSubsystem = trieurSubsystem;
        addRequirements(trieurSubsystem);
    }


    /**
     * Initiates the rotation by commanding the subsystem to move to the target position.
     */
    @Override
    public void initialize() {
        trieurSubsystem.moulinRevolution();
    }
    /**
     * The command is finished when the subsystem indicates that motor power should be cut.
     *
     * @return True if the command should end.
     */
    @Override
    public boolean isFinished() {
        return trieurSubsystem.shouldMoulinStopPower();
    }


}
