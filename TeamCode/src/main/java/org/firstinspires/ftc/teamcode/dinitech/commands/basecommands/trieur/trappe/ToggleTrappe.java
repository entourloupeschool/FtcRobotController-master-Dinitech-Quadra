package org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.trappe;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
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
    private final GamepadSubsystem gamepadSubsystem;

    /**
     * Creates a new ToggleTrappe command.
     *
     * @param trieurSubsystem  The sorter subsystem to control.
     * @param gamepadSubsystem The gamepad subsystem for providing haptic feedback.
     */
    public ToggleTrappe(TrieurSubsystem trieurSubsystem, GamepadSubsystem gamepadSubsystem){
        this.trieurSubsystem = trieurSubsystem;
        this.gamepadSubsystem = gamepadSubsystem;
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
            // Provide haptic feedback to confirm the action
            Gamepad.RumbleEffect customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                    .addStep(0.5, 0.5, 100) // A short, noticeable rumble
                    .build();
            gamepadSubsystem.customRumble(customRumbleEffect, 1);
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
