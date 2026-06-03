package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.finger;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * An instant command that toggles the state of the finger.
 * <p>
 * When scheduled, this command checks if the finger is currently open.
 * If it is, it closes it. If it's closed, it opens it.
 */
public class ToggleFinger extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;

    /**
     * Creates a new ToggleFinger command.
     *
     * @param trieurSubsystem  The sorter subsystem to control.
     */
    public ToggleFinger(TrieurSubsystem trieurSubsystem){
        this.trieurSubsystem = trieurSubsystem;
        addRequirements(trieurSubsystem);
    }

    /**
     * Toggles the finger's state.
     */
    @Override
    public void initialize(){
        trieurSubsystem.toggleFinger();
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
