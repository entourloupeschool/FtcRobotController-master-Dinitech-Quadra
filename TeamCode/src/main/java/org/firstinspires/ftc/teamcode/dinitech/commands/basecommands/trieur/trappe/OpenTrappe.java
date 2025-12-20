package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * An instant command that opens the trappe (trapdoor) on the sorter.
 * <p>
 * When scheduled, this command calls the {@code openTrappe()} method on the
 * {@link TrieurSubsystem} and finishes immediately.
 */
public class OpenTrappe extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;

    /**
     * Creates a new OpenTrappe command.
     *
     * @param trieurSubsystem The sorter subsystem to control.
     */
    public OpenTrappe(TrieurSubsystem trieurSubsystem){
        this.trieurSubsystem = trieurSubsystem;
        addRequirements(trieurSubsystem);
    }

    /**
     * Opens the trappe.
     */
    @Override
    public void initialize(){
        trieurSubsystem.openTrappe();
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
