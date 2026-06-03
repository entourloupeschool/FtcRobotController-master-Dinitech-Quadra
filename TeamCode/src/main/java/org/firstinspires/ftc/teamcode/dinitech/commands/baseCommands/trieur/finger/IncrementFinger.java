package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.finger;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * An instant command that makes a small, incremental adjustment to the finger's position.
 * <p>
 * This is useful for manual fine-tuning of the finger's angle. When scheduled, it calls
 * the {@code incrFinger()} method on the {@link TrieurSubsystem} with a specified increment
 * value and then finishes immediately.
 */
public class IncrementFinger extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;
    private final double increment;

    /**
     * Creates a new IncrementFinger command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     * @param increment       The amount (in servo degrees) to rotate the finger by. Can be positive or negative.
     */
    public IncrementFinger(TrieurSubsystem trieurSubsystem, double increment){
        this.trieurSubsystem = trieurSubsystem;
        this.increment = increment;
        addRequirements(trieurSubsystem);
    }

    /**
     * Applies the incremental rotation to the finger.
     */
    @Override
    public void initialize(){
        trieurSubsystem.incrFinger(increment);
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
