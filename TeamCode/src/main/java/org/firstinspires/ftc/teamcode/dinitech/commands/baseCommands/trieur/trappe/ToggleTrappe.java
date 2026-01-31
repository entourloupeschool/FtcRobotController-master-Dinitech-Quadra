package org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.trieur.trappe;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;

/**
 * An instant command that toggles the state of the trappe (trapdoor).
 * <p>
 * When scheduled, this command checks if the trappe is currently open.
 * If it is, it closes it. If it's closed, it opens it and provides a short
 * rumble on the driver's gamepad for haptic feedback.
 */
public class ToggleTrappe extends CommandBase {
    private final TrieurSubsystem trieurSubsystem;

    /**
     * Creates a new ToggleTrappe command.
     *
     * @param trieurSubsystem  The sorter subsystem to control.
     */
    public ToggleTrappe(TrieurSubsystem trieurSubsystem){
        this.trieurSubsystem = trieurSubsystem;
    }

    /**
     * Toggles the trappe's state and provides haptic feedback.
     */
    @Override
    public void initialize(){
        if (trieurSubsystem.isTrappeOpen()){
            trieurSubsystem.closeTrappe();
        } else {
            trieurSubsystem.openTrappe();
        }
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
