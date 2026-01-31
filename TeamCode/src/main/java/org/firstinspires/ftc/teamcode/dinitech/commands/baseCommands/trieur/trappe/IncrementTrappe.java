package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * An instant command that makes a small, incremental adjustment to the trappe's position.
 * <p>
 * This is useful for manual fine-tuning of the trappe's angle. When scheduled, it calls
 * the {@code incrTrappe()} method on the {@link TrieurSubsystem} with a specified increment
 * value and then finishes immediately.
 */
public class IncrementTrappe extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;
    private final double increment;

    /**
     * Creates a new IncrementTrappe command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     * @param increment       The amount (in servo degrees) to rotate the trappe by. Can be positive or negative.
     */
    public IncrementTrappe(TrieurSubsystem trieurSubsystem, double increment){
        this.trieurSubsystem = trieurSubsystem;
        this.increment = increment;
    }

    /**
     * Applies the incremental rotation to the trappe.
     */
    @Override
    public void initialize(){
        trieurSubsystem.incrTrappe(increment);
    }

    /**
     * This command is always finished immediately.
     *
     * @return Always returns true.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
