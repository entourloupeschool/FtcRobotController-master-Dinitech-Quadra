package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.finger;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * An instant command that closes the finger on the sorter.
 * <p>
 * When scheduled, this command calls the {@code closeFinger()} method on the
 * {@link TrieurSubsystem} and finishes immediately.
 */
public class CloseFinger extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;

    /**
     * Creates a new CloseFinger command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public CloseFinger(TrieurSubsystem trieurSubsystem){
        this.trieurSubsystem = trieurSubsystem;
        addRequirements(trieurSubsystem);
    }

    /**
     * Closes the finger.
     */
    @Override
    public void initialize(){
        trieurSubsystem.closeFinger();
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
