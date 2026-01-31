package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * An instant command that closes the trappe (trapdoor) on the sorter.
 * <p>
 * When scheduled, this command calls the {@code closeTrappe()} method on the
 * {@link TrieurSubsystem} and finishes immediately.
 */
public class CloseTrappe extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;

    /**
     * Creates a new CloseTrappe command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public CloseTrappe(TrieurSubsystem trieurSubsystem){
        this.trieurSubsystem = trieurSubsystem;
    }

    /**
     * Closes the trappe.
     */
    @Override
    public void initialize(){
        trieurSubsystem.closeTrappe();
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
